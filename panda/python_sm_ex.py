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

DEG_TO_RAD = math.pi / 180.0

class State(Enum):
  INIT = auto()
  PICK_WAIT = auto()
  PRE_PICK = auto()
  PICK = auto()
  POST_PICK = auto()
  PRE_PLACE = auto()
  PLACE = auto()
  FINISH = auto()


# CHANGE TO TRUE TO RUN IN SIMULATION
simulation = False

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


def camera_to_base(camera_point, ee_position, ee_orientation):

    offset = np.array([-0.01, -0.05, -0.09]) # Camera frame to ee frame
    camera_point += offset

    camera_point = np.array([-camera_point[1], camera_point[0], camera_point[2]])

    p_cam = np.asarray(camera_point, dtype=float)
    t_ee = np.asarray(ee_position, dtype=float)
    R_be = np.asarray(ee_orientation, dtype=float)

    R_be = R_be.T

    # base_point = R_be * p_cam + t_ee
    base_point = R_be.dot(p_cam) + t_ee
    return base_point

def parse_xyz_string(s):
    # find all floats in the string
    pattern = r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?'
    nums = re.findall(pattern, s)
    if len(nums) != 3:
        raise ValueError(f"Expected 3 numbers in '{s}', got {len(nums)}")
    return np.array([float(n) for n in nums], dtype=float)


redis_keys = RedisKeys()

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



init_position = np.array([0.32, 0.50, 0.4])
pre_pick_position = np.array([0.32, 0.50, 0.4])
#pick_position = np.array([0.33, 0.49, 0.18])

pre_place_position = np.array([0.04, 0.54, 0.34])
place_position = np.array([0.04, 0.54, 0.185])

goal_position = init_position
goal_orientation = np.array([[1.0,0,0],[0,-1.0,0],[0,0,-1.0]])

pos_error = 1
ori_error = 1

try:
  while True:
    loop_time += dt
    time.sleep(max(0, loop_time - (time.perf_counter_ns() * 1e-9 - init_time)))
    
    # check the active controller is the cartesian one - SKIPPED
    active_controller = redis_client.get(redis_keys.active_controller).decode("utf-8")
    if active_controller != "cartesian_controller":
        print("Exiting, active controller is not cartesian_controller")
        exit(0)
    
    # read robot state - SKIPPED
    current_position = np.array(json.loads(redis_client.get(redis_keys.cartesian_task_current_position)))
    current_orientation = np.array(json.loads(redis_client.get(redis_keys.cartesian_task_current_orientation)))

    #print(current_position)
    # Get error - SKIPPED
    pos_error = np.linalg.norm(goal_position - current_position)
    ori_error = np.linalg.norm(goal_orientation - current_orientation)

    # state machine
    if state == State.INIT:

      if pos_error < 1e-2 and ori_error < 1e-1:
        goal_position = pre_pick_position
        state = State.PICK_WAIT


    elif state == State.PICK_WAIT:
      
      #print(flag)

      if pos_error < 1e-2 and ori_error < 1e-1:
        
        flag = "False"
        flag = redis_client.get(redis_keys.object_present_flag).decode("utf-8")

        if flag == "True":
            camera_point_str = redis_client.get(redis_keys.object_location).decode("utf-8")
            camera_point = parse_xyz_string(camera_point_str)
            #print(camera_point)
            block_pos = camera_to_base(camera_point, current_position, current_orientation)
            time.sleep(3.0)
            print(block_pos)
            goal_position = np.array([block_pos[0], block_pos[1], 0.25])
            time_start = time.time()
            state = State.PRE_PICK

    elif state == State.PRE_PICK:
      
      if pos_error < 1e-2 and ori_error < 1e-1:
        flag = "False"
        flag = redis_client.get(redis_keys.object_present_flag).decode("utf-8")
        if time.time() - time_start > 8.0:
            goal_position = pre_pick_position
            state = State.PICK_WAIT
        if flag == "True":
            camera_point_str = redis_client.get(redis_keys.object_location).decode("utf-8")
            camera_point = parse_xyz_string(camera_point_str)
            block_pos = camera_to_base(camera_point, current_position, current_orientation)
            goal_position = block_pos
            state = State.PICK

        
    elif state == State.PICK:
      
      if pos_error < 1e-2 and ori_error < 1e-1:
        redis_client.set(redis_keys.gripper_control, "g")
        time.sleep(2.0)
        goal_position = pre_pick_position
        state = State.POST_PICK


    elif state == State.POST_PICK:

      if pos_error < 1e-2 and ori_error < 1e-1:
        goal_position = pre_place_position
        state = State.PRE_PLACE


    elif state == State.PRE_PLACE:

      if pos_error < 1e-2 and ori_error < 1e-1:
        goal_position = place_position
        state = State.PLACE


    elif state == State.PLACE:
      if pos_error < 1e-2 and ori_error < 1e-1:
        time.sleep(0.5)
        redis_client.set(redis_keys.gripper_control, "o")
        time.sleep(2.0)
        goal_position = pre_pick_position
        state = State.PICK_WAIT


    elif state == State.FINISH:
        redis_client.set(redis_keys.cartesian_task_goal_position, json.dumps(pre_place_position.tolist()))
        redis_client.set(redis_keys.cartesian_task_goal_orientation, json.dumps(goal_orientation.tolist()))
        print("Finished")
        exit(0)


    if state != state.FINISH:
        redis_client.set(redis_keys.cartesian_task_goal_position, json.dumps(goal_position.tolist()))
        redis_client.set(redis_keys.cartesian_task_goal_orientation, json.dumps(goal_orientation.tolist()))


        


except KeyboardInterrupt:
  print("Keyboard interrupt")
  pass
except Exception as e:
  print(e)
  pass