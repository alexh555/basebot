/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <SaiModel.h>
#include "SaiPrimitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;
using namespace SaiPrimitives;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#include "redis_keys.h"

// States 
// enum State {
// 	POSTURE = 0, 
// 	MOTION
// };
// Python runs state machine now

int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_net.urdf";

	// initial state 
	// int state = POSTURE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	const string control_link = "end-effector";
	const Vector3d control_point = Vector3d(0, 0, 0.17);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setPosControlGains(400, 40, 0);
	pose_task->setOriControlGains(400, 40, 0);
	pose_task->disableVelocitySaturation();
	pose_task->disableInternalOtg();

	Vector3d ee_pos;
	Matrix3d ee_ori;

	// gripper partial joint task 
	// MatrixXd gripper_selection_matrix = MatrixXd::Zero(2, robot->dof());
	// gripper_selection_matrix(0, 7) = 1;
	// gripper_selection_matrix(1, 8) = 1;
	// auto gripper_task = std::make_shared<SaiPrimitives::JointTask>(robot, gripper_selection_matrix);
	// gripper_task->setDynamicDecouplingType(SaiPrimitives::DynamicDecouplingType::IMPEDANCE);
	// double kp_gripper = 5e3;
	// double kv_gripper = 1e2;
	// gripper_task->setGains(kp_gripper, kv_gripper, 0);
	// gripper_task->setGains(kp_gripper, kv_gripper, 0);

	// joint task
	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(400, 40, 0);

	VectorXd q_desired(dof);
	q_desired.head(7) << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_desired.head(7) *= M_PI / 180.0;
	//q_desired.tail(2) << 0.04, -0.04;
	joint_task->setGoalPosition(q_desired);

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	// Ensure pose task initialized
	pose_task->reInitializeTask();
	cout << "Initialized pose_task" << endl;

	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

		// Read goal from Python, if available
		if(redis_client.exists(CATCHING_POS) && redis_client.exists(CATCHING_ORI)) {

			// Update goals
			ee_pos = redis_client.getEigen(CATCHING_POS);
			ee_ori = redis_client.getEigen(CATCHING_ORI);

			// Update torques
			pose_task->setGoalPosition(ee_pos);
			pose_task->setGoalOrientation(ee_ori);

			// Command torques
			N_prec.setIdentity(); // update task model
			pose_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			cout << command_torques.transpose() << endl;
		} else {
			// No goal, stay at zero
			cout << "NO GOAL" << endl;
			command_torques = VectorXd::Zero(dof);

		}
		

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
