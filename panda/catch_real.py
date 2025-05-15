import numpy as np
import time
import json
import redis
import time
import math
import re
from enum import Enum, auto
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R

from catching_in_sim import get_catch_point, velo_to_ori

DEG_TO_RAD = math.pi / 180.0

class State(Enum):
  INIT = auto()
  CATCH = auto()
  FINISH = auto()


# CHANGE TO TRUE TO RUN IN SIMULATION
simulation = False

# *** REDIS KEY SETUP ***
@dataclass
class RedisKeys:

  if simulation:
    prefix = "Panda"
  else:
    prefix = "FrankaRobot"

  cartesian_task_goal_position: str = "sai::controllers::" + prefix + "::cartesian_controller::cartesian_task::goal_position"
  cartesian_task_goal_orientation: str = "sai::controllers::" + prefix + "::cartesian_controller::cartesian_task::goal_orientation"
  cartesian_task_current_position: str = "sai::controllers::" + prefix + "::cartesian_controller::cartesian_task::current_position"
  cartesian_task_current_orientation: str = "sai::controllers::" + prefix + "::cartesian_controller::cartesian_task::current_orientation"
  active_controller: str = "sai::controllers::" + prefix + "::active_controller_name"
  config_file_name: str = "::sai-interfaces-webui::config_file_name"
  gripper_task_goal_position: str = "sai::controllers::" + prefix + "::cartesian_controller::gripper_fingers::goal_position"
  #object_current_position: str = "sai::sensors::Box::object_pose"

  gripper_control: str = "sai::FrankaRobot::gripper::mode"
  object_present_flag: str = "realsense_demo_object_present_flag"
  object_location: str = "realsense_demo_cube_pos"

  # CATCHING KEYS
  initial_ball_position: str = "sai::controllers::" + prefix + "::cartesian_controller::cartesian_task::ball_position"
  initial_ball_velocity: str = "sai::controllers::" + prefix + "::cartesian_controller::cartesian_task::ball_velocity"

redis_keys = RedisKeys()

# *** CONFIG AND CONTROLLER SETUP ***
config_file_for_this_example = "single_panda.xml"
controller_to_use = "cartesian_controller"

#place_goal_position_left = np.array([0.4, -0.3, 0.225])
#place_goal_position_right = np.array([0.45, 0.35, 0.325])

# redis client
redis_client = redis.Redis()

# check that the config file is correct
config_file_name = redis_client.get(redis_keys.config_file_name).decode("utf-8")
if config_file_name != config_file_for_this_example:
  print("This example is meant to be used with the config file: ", config_file_for_this_example)
  exit(0)

# set the correct active controller
while redis_client.get(redis_keys.active_controller).decode("utf-8") != controller_to_use:
	redis_client.set(redis_keys.active_controller, controller_to_use)
   
# loop at 100 Hz
loop_time = 0.0
dt = 0.01
internal_step = 0
state = State.INIT

time.sleep(0.01)
init_time = time.perf_counter_ns() * 1e-9

time_start = 0


# *** POSITION INITIALIZATION ***
init_position = np.array([0.1, -0.50, 0.5]) # Where to put net center at start
init_orientation = np.array([[1.0,0,0],[0,-1.0,0],[0,0,-1.0]])

goal_position = init_position # Set as where to start
goal_orientation = init_orientation

current_ball_position = np.array([0.0, 0.0, 0.0])
current_ball_velocity = np.array([0.0, 0.0, 0.0])

pos_error = 1
ori_error = 1

try:
  while True:
    loop_time += dt
    time.sleep(max(0, loop_time - (time.perf_counter_ns() * 1e-9 - init_time)))
    
    # check the active controller is the cartesian one
    active_controller = redis_client.get(redis_keys.active_controller).decode("utf-8")
    if active_controller != "cartesian_controller":
        print("Exiting, active controller is not cartesian_controller")
        exit(0)
    
    # read robot state
    current_position = np.array(json.loads(redis_client.get(redis_keys.cartesian_task_current_position)))
    current_orientation = np.array(json.loads(redis_client.get(redis_keys.cartesian_task_current_orientation)))

    #print(current_position)

    pos_error = np.linalg.norm(goal_position - current_position)
    ori_error = np.linalg.norm(goal_orientation - current_orientation)

    # state machine
    if state == State.INIT:

        # Ensure got to intial position
        if pos_error < 1e-2 and ori_error < 1e-1:

            # Now, check if ball info is available
            if( redis_client.exists(redis_keys.initial_ball_position) and redis_client.exists(redis_keys.initial_ball_velocity) ):
    
                # Read in init ball 
                bpos_str = redis_client.get(redis_keys.initial_ball_position).decode("utf-8")
                bvel_str = redis_client.get(redis_keys.initial_ball_velocity).decode("utf-8")

                current_ball_position = np.array([float(p) for p in bpos_str.strip("[]").split(",")])  # Initial position (m)
                current_ball_velocity = np.array([float(v) for v in bvel_str.strip("[]").split(",")])   # Initial velocity (m/s)

                # Generate resulting catch point
                new_position, new_orientation = get_catch_point(current_ball_position, current_ball_velocity)

                # Only move to catch if it is diff than initial
                delta_pos = new_position - init_position
                if ( np.linalg.norm(delta_pos) > 1e-6):
                    goal_position = new_position
                    goal_orientation = new_orientation

                    print(f"NEW GOAL POS: {goal_position}")
                    print(f"NEW GOAL ORI: {goal_orientation}")
                    
                    # State transition
                    state = State.CATCH


    elif state == State.CATCH:
        # Check if got to goal catching position
        if pos_error < 1e-2 and ori_error < 1e-1:
           state = State.FINISH # If so, stop
    
    elif state == State.FINISH:
        pass # For now, stay here permanently



    # *** ALWAYS TRANSMIT GOAL, unless at end ***
    if state != State.FINISH:
        redis_client.set(redis_keys.cartesian_task_goal_position, json.dumps(goal_position.tolist()))
        redis_client.set(redis_keys.cartesian_task_goal_orientation, json.dumps(goal_orientation.tolist()))


        


except KeyboardInterrupt:
  print("Keyboard interrupt")
  pass
except Exception as e:
  print(e)
  pass