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


bool simulation = false;



int main() {


	if (simulation){
		cout << "SIMULATION TRUE" << endl;
		JOINT_ANGLES_KEY = "sai::sim::PANDA::sensors::q";
		JOINT_VELOCITIES_KEY = "sai::sim::PANDA::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY = "sai::sim::PANDA::actuators::fgc";
	} else{
		cout << "SIMULATION FALSE" << endl;
		JOINT_TORQUES_COMMANDED_KEY = "sai::commands::FrankaRobot::control_torques";
		JOINT_VELOCITIES_KEY = "sai::sensors::FrankaRobot::joint_velocities";
		JOINT_ANGLES_KEY = "sai::sensors::FrankaRobot::joint_positions";
		MASS_MATRIX_KEY = "sai::sensors::FrankaRobot::model::mass_matrix";
	}


	// Location of URDF files specifying world and robot information
	static const string robot_file = "../../urdf/panda_arm_net.urdf";

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


	MatrixXd M = robot->M();
	if(!simulation) {
		M = redis_client.getEigen(MASS_MATRIX_KEY);
		// bie addition
		M(4,4) += 0.2;
		M(5,5) += 0.2;
		M(6,6) += 0.2;
	}
	robot->updateModel(M);

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	const string control_link = "end-effector";
	const Vector3d control_point = Vector3d(0, 0, 0.17);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	//auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);

	std::vector<Vector3d> controlled_pos_dirs;
	controlled_pos_dirs.push_back(Vector3d(1, 0, 0));
	controlled_pos_dirs.push_back(Vector3d(0, 1, 0));
	controlled_pos_dirs.push_back(Vector3d(0, 0, 1));
	std::vector<Vector3d> controlled_ori_dirs;
	controlled_ori_dirs.push_back(Vector3d(0, 1, 0));
	controlled_ori_dirs.push_back(Vector3d(0, 0, 1));
	auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, controlled_pos_dirs, controlled_ori_dirs, compliant_frame);

	pose_task->setPosControlGains(200, 20, 0);
	pose_task->setOriControlGains(200, 20, 0);

	/* VELOCITY SAT*/
	//pose_task->disableVelocitySaturation();
	pose_task->enableVelocitySaturation(0.7, M_PI/3.0); // Was 0.7 and pi/3

	/* OTG */
	// pose_task->enableInternalOtgAccelerationLimited(
	// 	1.50,
	// 	2.0, 
	// 	M_PI / 3.0,
	// 	2.0 * M_PI);
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
	joint_task->setGains(100, 20, 0);

	VectorXd q_desired(dof);
	//q_desired.head(7) << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	//q_desired.head(7) << 130.0, 45.0, 160.0, -140.0, -45.0, 140.0 -45.0;
	q_desired.head(7) << -60.0, 0.0, -15.0, -100.0, 0.0, 100.0 -60.0;
	q_desired.head(7) *= M_PI / 180.0;
	//q_desired.head(7) = robot->q();
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

		M = robot->M();
		if(!simulation) {
			M = redis_client.getEigen(MASS_MATRIX_KEY);
			// bie addition
			M(4,4) += 0.2;
			M(5,5) += 0.2;
			M(6,6) += 0.2;
		}
		robot->updateModel(M);


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

			//cout << command_torques.transpose() << endl;
		} else {
			// No goal, stay at zero
			cout << "NO GOAL" << endl;
			command_torques = VectorXd::Zero(dof);

		}
		

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		// Also share robot position
		redis_client.setEigen(ROBOT_EE_POS, robot->position(control_link, control_point));

		Matrix3d rot_in_link;
        rot_in_link << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
		redis_client.setEigen(ROBOT_EE_ORI, robot->rotation(control_link, rot_in_link));

		// print current position
		VectorXd q_cur = robot->q();
		q_cur *= 180.0/M_PI;
		cout << q_cur.transpose() << endl;
		
	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
