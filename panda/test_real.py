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

# CHANGE TO TRUE TO RUN IN SIMULATION
simulation = False

# *** REDIS SETUP ***
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
redis_client = redis.Redis()


# *** POSITION INITIALIZATION ***
test_i_ball_position = np.array([5.0, -0.5, 1.0])
test_i_ball_velocity = np.array([-10.0, 0.0, 1.0])

try:

    # Pause until ready
    print("Starting test script!")
    input("Press ENTER to move to test pos...\n")

    # Now, send test
    redis_client.set(redis_keys.initial_ball_position, json.dumps(test_i_ball_position.tolist()))
    redis_client.set(redis_keys.initial_ball_velocity, json.dumps(test_i_ball_velocity.tolist()))

    time.sleep(1.0) # Ensure other script got it

    # Clean up, so catch_real doesn't always move
    redis_client.delete(redis_keys.initial_ball_position)
    redis_client.delete(redis_keys.initial_ball_velocity)

    print("Ball keys cleared!")

except KeyboardInterrupt:
    print("Keyboard interrupt")
    pass
except Exception as e:
    print(e)
    pass