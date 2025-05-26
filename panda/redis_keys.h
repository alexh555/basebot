/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 * 
 */

std::string JOINT_ANGLES_KEY = "sai::sim::panda::sensors::q";
std::string JOINT_VELOCITIES_KEY = "sai::sim::panda::sensors::dq";
std::string JOINT_TORQUES_COMMANDED_KEY = "sai::sim::panda::actuators::fgc";



const std::string CONTROLLER_RUNNING_KEY = "sai::sim::panda::controller";

const std::string GRIPPER_JOINT_ANGLES_KEY = "sai::sim::panda_gripper::sensors::q";
const std::string GRIPPER_JOINT_VELOCITIES_KEY = "sai::sim::panda_gripper::sensors::dq";

/* Simulation Catch Keys */
const std::string INITIAL_BALL_POS = "sai::sim::ball::initial::position";
const std::string INITIAL_BALL_VELO = "sai::sim::ball::initial::velocity";
const std::string CATCHING_POS = "sai::sim::panda::catching::position";
const std::string CATCHING_ORI = "sai::sim::panda::catching::orientation";

const std::string ROBOT_EE_POS = "sai::controllers::Panda::cartesian_controller::cartesian_task::current_position";
const std::string ROBOT_EE_ORI = "sai::controllers::Panda::cartesian_controller::cartesian_task::current_orientation";

const std::string TRUE_BALL_POS = "sai2::optitrack::rigid_body_pos::4";
const std::string TRUE_BOT_POS = "sai2::optitrack::rigid_body_pos::5";

 std::string MASS_MATRIX_KEY;