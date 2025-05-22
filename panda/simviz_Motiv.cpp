/**
 * @file simviz.cpp
 * @brief Simulation and visualization of panda robot with 1 DOF gripper 
 * 
 */

#include <math.h>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <random>

#include "SaiGraphics.h"
#include "SaiModel.h"
#include "SaiSimulation.h"
#include "SaiPrimitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"

bool fSimulationRunning = true;
void sighandler(int){fSimulationRunning = false;}

#include "redis_keys.h"

using namespace Eigen;
using namespace std;

// mutex and globals
VectorXd ui_torques;
mutex mutex_torques, mutex_update;

// specify urdf and robots 
static const string robot_name = "panda_arm_net";
static const string camera_name = "camera_fixed";

// dynamic objects information
const vector<std::string> object_names = {"ball"};
vector<Affine3d> object_poses;
vector<VectorXd> object_velocities;
const int n_objects = object_names.size();

// simulation thread
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim);

int main() {
	SaiModel::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_net.urdf";
	static const string world_file = string(PANDA_FOLDER) + "/world.urdf";
	std::cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = std::make_shared<SaiGraphics::SaiGraphics>(world_file, camera_name, false);
	graphics->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 	
	// graphics->showLinkFrame(true, robot_name, "link7", 0.15);  // can add frames for different links
	// graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes 
	graphics->addUIForceInteraction(robot_name);

	// load robots
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);
	// robot->setQ();
	// robot->setDq();
	robot->updateModel();
	ui_torques = VectorXd::Zero(robot->dof());

	// load simulation world
	auto sim = std::make_shared<SaiSimulation::SaiSimulation>(world_file, false);
	sim->setJointPositions(robot_name, robot->q());
	sim->setJointVelocities(robot_name, robot->dq());

	// fill in object information 
	for (int i = 0; i < n_objects; ++i) {
		object_poses.push_back(sim->getObjectPose(object_names[i]));
		object_velocities.push_back(sim->getObjectVelocity(object_names[i]));
	}	

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// init redis client values 
	redis_client.setEigen(JOINT_ANGLES_KEY, robot->q()); 
	redis_client.setEigen(JOINT_VELOCITIES_KEY, robot->dq()); 
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * robot->q());

	// start simulation thread
	thread sim_thread(simulation, sim);

	// while window is open:
	while (graphics->isWindowOpen() && fSimulationRunning) {
        graphics->updateRobotGraphics(robot_name, redis_client.getEigen(JOINT_ANGLES_KEY));
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				graphics->updateObjectGraphics(object_names[i], object_poses[i]);
			}
		}
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			ui_torques = graphics->getUITorques(robot_name);
		}
	}

    // stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
// Helper for transforming ball position
static Vector3d transform_ball_pos(Vector3d pos_from_redis)
{
	Vector3d orig_pos = pos_from_redis;
	Vector3d transf_pos = orig_pos;

	// VERSION 1
    // transf_pos[0] = orig_pos[2] + 5 - 0.5; // Remove second term to to align with blue X
    // transf_pos[1] = orig_pos[0] - 0.75; //- 0.7; // Remove second term to to align with blue X
    // transf_pos[2] = orig_pos[1] - 0.4;

	// V2 - Someone changed origin
	transf_pos[0] = -orig_pos[1] + 5 - 0.5; //
    transf_pos[1] = orig_pos[0] - 0.7; //
    transf_pos[2] = orig_pos[2] - 0.4;
	
	return transf_pos;
}

// Simulator
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim) {
	// fSimulationRunning = true;

    // create redis client
    auto redis_client = SaiCommon::RedisClient();
    redis_client.connect();

	// create a timer
	double sim_freq = 2000;
	SaiCommon::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);
    sim->enableGravityCompensation(true);
	sim->enableJointLimits(robot_name);

	// GET ARM TO STARTING POSITION
	Vector3d velo_to_start(-100.0, 0.0, 0.0); // basically move straight in x
	Vector3d pos_to_start(3.0, -0.5, 0.5); // So arm will start here
	redis_client.setEigen(INITIAL_BALL_POS, pos_to_start);
	redis_client.setEigen(INITIAL_BALL_VELO, velo_to_start);

	long step_count = 0; // For starting ball later
	bool ball_launched = false;

	// Wait for start
	// std::cout << "Press ENTER to start simulation..." << std::endl;
	// std::cin.ignore();

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		step_count++; // To start ball later

		VectorXd control_torques = redis_client.getEigen(JOINT_TORQUES_COMMANDED_KEY);
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(robot_name, control_torques + ui_torques);
		}
		sim->integrate();
        redis_client.setEigen(JOINT_ANGLES_KEY, sim->getJointPositions(robot_name));
        redis_client.setEigen(JOINT_VELOCITIES_KEY, sim->getJointVelocities(robot_name));

		// Launch ball after 2 seconds
		// if(!ball_launched && step_count > (2*sim_freq))
		// {
		// 	// NOW, create ball
		// 	std::random_device rd;            // Setup random
		// 	std::mt19937 gen(rd());           // random number engine
		// 	std::uniform_real_distribution<> dis(0.0, 0.1);  // range [0.0, 0.4] 

		// 	float y_rand_offset = -5*dis(gen);
		// 	float z_rand_offset = 4*dis(gen);
		// 	float v_y_offset = 4*dis(gen);
		// 	float v_z_offset = 2*dis(gen);

		// 	Vector3d init_linear_velocity(-10.0, -0.2 + v_y_offset, 0.9 + v_z_offset);  // TODO: adjust as needed
		// 	Affine3d init_ball_pose = Eigen::Affine3d::Identity();
		// 	init_ball_pose.translation() = Eigen::Vector3d(5.0, -0.25 + y_rand_offset, 0.9 + z_rand_offset);

		// 	sim->setObjectVelocity(object_names[0], init_linear_velocity);
		// 	sim->setObjectPose(object_names[0], init_ball_pose);
		// 	// sim->setObjectAngularVelocity(object_names[0], init_angular_velocity);

		// 	// POST INITIAL BALL STATE
		// 	redis_client.setEigen(INITIAL_BALL_POS, init_ball_pose.translation());
		// 	redis_client.setEigen(INITIAL_BALL_VELO, init_linear_velocity);

		// 	cout << "Init ball position = " << init_ball_pose.translation() << endl;
		// 	ball_launched = true; // Don't repeat
		// }


		// update object information USING OPTITRACK
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				// object_poses[i] = sim->getObjectPose(object_names[i]);
				// object_velocities[i] = sim->getObjectVelocity(object_names[i]);

				if (redis_client.exists(TRUE_BALL_POS)) {
					Vector3d ball_pos_to_print = redis_client.getEigen(TRUE_BALL_POS);  // ← replace with correct Redis key
					ball_pos_to_print = transform_ball_pos(ball_pos_to_print);
					object_poses[i].translation() = ball_pos_to_print;
					// Optionally zero the orientation:
					object_poses[i].linear() = Matrix3d::Identity();

					cout << "Ball pos = " << ball_pos_to_print.transpose() << endl;

				} else {
					cout << "BALL ERROR" << endl;
				}

				
			}

			// Post ball info, if launched
			// if (ball_launched)
			// {
			// 	Vector3d ball_pos_to_print = object_poses[0].translation();
			// 	// cout << "Ball pos = " << ball_pos_to_print.transpose() << endl;
			// 	//redis_client.setEigen(TRUE_BALL_POS, ball_pos_to_print);
			// }
			
		}
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}