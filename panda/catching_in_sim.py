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
X_CUTOFF = 0.1

# State var
class State(Enum):
  INIT = auto()
  CATCH = auto()

# CHANGE TO TRUE TO RUN IN SIMULATION
simulation = True

# Redis keys
@dataclass
class RedisKeys:

    # These are only keys you should need for initial catch
    initial_ball_position: str = "sai::sim::ball::initial::position"
    initial_ball_velocity: str = "sai::sim::ball::initial::velocity"
    catching_position: str = "sai::sim::panda::catching::position"
    catching_orientation: str = "sai::sim::panda::catching::orientation"


# Trajectory
def get_catch_point(pbi, vbi): 
    """
    Determines point where robot should catch ball given initial ball velocity and position
    """

    # Constants
    mass_ounces = 5
    mass_kg = mass_ounces * 0.0283495  # Convert ounces to kg
    gravity = np.array([0, 0, -9.8])  # Acceleration due to gravity (m/s^2)
    # air_density = 1.225  # Air density at sea level (kg/m^3)
    # drag_coefficient = 0.3  # Approximate drag coefficient for a baseball
    # radius = 0.037  # Approximate radius of a baseball (m)
    # cross_sectional_area = np.pi * radius**2

    # Initial conditions
    initial_position = pbi  # Initial position (m)
    initial_velocity = vbi   # Initial velocity (m/s)

    # Time parameters
    time_step = 0.01  # Time step for simulation (s)
    max_time = 10      # Maximum simulation time (s)

    # Lists to store position and velocity at each time step -
    # positions = [initial_position]
    # velocities = [initial_velocity]
    # time_points = [0]

    # Initialize sim vars
    current_position = initial_position
    current_velocity = initial_velocity
    current_time = 0

    # Simulation loop - REPEAT UNTIL WITHIN X WINDOW, OR MAX TIME REACHED
    while current_position[0] > 0.1 and current_time < max_time:
        # Calculate air drag force - SKIP INITIALLY
        # velocity_magnitude = np.linalg.norm(current_velocity)
        # drag_force_magnitude = 0.5 * air_density * drag_coefficient * cross_sectional_area * velocity_magnitude**2
        # drag_force = -drag_force_magnitude * (current_velocity / velocity_magnitude)

        # Calculate net force
        net_force = mass_kg * gravity # + drag_force # SKIPPED DRAG FOR NOW

        # Calculate acceleration
        acceleration = net_force / mass_kg

        # Update velocity and position using Euler's method
        current_velocity = current_velocity + acceleration * time_step
        current_position = current_position + current_velocity * time_step
        current_time += time_step

        # Store data - SKIPPED
        # velocities.append(current_velocity)
        # positions.append(current_position)
        # time_points.append(current_time)

    # Confirm final point reachable
    print(f"Final catch pos = {current_position}")

    if ( abs(current_position[0]) < 0.25 and
            current_position[1] < -0.25 and
            current_position[1] > -0.75 and
            current_position[2] > 0.1 and
            current_position[2] < 1.1 ):

        next_ori = velo_to_ori(current_velocity)

        return current_position, next_ori
    else:
        print("!!!ERROR!!! Final catchable pos not reachable")
        return init_position, init_orientation # Maintain starting position, defined below
  

def velo_to_ori(velocity_vec):
    """
    Computes robot orientation given catch point velocity of ball
    """

    x_des = -velocity_vec / np.linalg.norm(velocity_vec) # Desired x axis should be anti-parallel to velocity

    # Calc Z - try to get close to [0, 0, -1]
    y_bias = np.array([0, -1, 0])
    y_des = y_bias - np.dot(y_bias, x_des) * x_des # project z_bias onto plane perp to x_des

    if np.linalg.norm(y_des) < 1e-6: # If x actually already along y_bias, then fall back to putting it along z
        y_des = np.cross(x_des, [0, 0, -1])
    
    y_des /= np.linalg.norm(y_des)

    # Finally, get valid y
    z_des = np.cross(x_des, y_des)

    # Now construct R
    R_des = np.column_stack([x_des, y_des, z_des])
    return R_des

# ***Redis, config, and controller name init - SKIPPED b/c shouldnt be necessary ***
redis_keys = RedisKeys()

#config_file_for_this_example = "single_panda.xml"
#controller_to_use = "cartesian_controller"

#place_goal_position_left = np.array([0.4, -0.3, 0.225])
#place_goal_position_right = np.array([0.45, 0.35, 0.325])

# redis client
redis_client = redis.Redis()

# # check that the config file is correct
# config_file_name = redis_client.get(redis_keys.config_file_name).decode("utf-8")
# if config_file_name != config_file_for_this_example:
#   print("This example is meant to be used with the config file: ", config_file_for_this_example)
#   exit(0)

# # set the correct active controller
# while redis_client.get(redis_keys.active_controller).decode("utf-8") != controller_to_use:
# 	redis_client.set(redis_keys.active_controller, controller_to_use)
   
# Initialize loop at 100 Hz
loop_time = 0.0
dt = 0.01
internal_step = 0
state = State.INIT

time.sleep(0.01)
init_time = time.perf_counter_ns() * 1e-9

time_start = 0

# Position initializations
init_position = np.array([0.1, -0.40, 0.5]) # Where to go at start
init_orientation = np.array([[1.0,0,0],[0,-1.0,0],[0,0,-1.0]])

goal_position = init_position # Set as where to start
goal_orientation = init_orientation

current_ball_position = np.array([0.0, 0.0, 0.0])
current_ball_velocity = np.array([0.0, 0.0, 0.0])
last_ball_position = current_ball_position
last_ball_velocity = current_ball_velocity

# *** MAIN LOOP ***
try:
  while True:
    loop_time += dt
    time.sleep(max(0, loop_time - (time.perf_counter_ns() * 1e-9 - init_time)))
    
    # check the active controller is the cartesian one - SKIPPED
    # active_controller = redis_client.get(redis_keys.active_controller).decode("utf-8")
    # if active_controller != "cartesian_controller":
    #     print("Exiting, active controller is not cartesian_controller")
    #     exit(0)
    
    # read robot state - SKIPPED
    # current_position = np.array(json.loads(redis_client.get(redis_keys.cartesian_task_current_position)))
    # current_orientation = np.array(json.loads(redis_client.get(redis_keys.cartesian_task_current_orientation)))

    # Error - SKIPPED
    #print(current_position)
    # pos_error = np.linalg.norm(goal_position - current_position)
    # ori_error = np.linalg.norm(goal_orientation - current_orientation)

    # state machine
    # if state == State.INIT:
      
    # Command initial position
    redis_client.set(redis_keys.catching_position, json.dumps(goal_position.tolist()))
    redis_client.set(redis_keys.catching_orientation, json.dumps(goal_orientation.tolist()))

    # Determine if ball trajectory available
    # FOR HARDWARE, THIS IF SHOULD BE BASED ON OPTITRACK
    # Also, should prob use state machine
    if( redis_client.exists(redis_keys.initial_ball_position) and redis_client.exists(redis_keys.initial_ball_velocity) ):
    
        # Read in init ball 
        bpos_str = redis_client.get(redis_keys.initial_ball_position).decode("utf-8")
        bvel_str = redis_client.get(redis_keys.initial_ball_velocity).decode("utf-8")

        current_ball_position = np.array([float(p) for p in bpos_str.strip("[]").split(",")])  # Initial position (m)
        current_ball_velocity = np.array([float(v) for v in bvel_str.strip("[]").split(",")])   # Initial velocity (m/s)

        # print("Got ball info!")
        # print(current_ball_position)
        # print(current_ball_velocity)
        # print(type(current_ball_position))
        # print(type(current_ball_velocity))

        # ***SIMULATE ENDING POSITION***
        if not np.allclose(current_ball_position, last_ball_position): # Only sim and update on change
            goal_position, goal_orientation = get_catch_point(current_ball_position, current_ball_velocity)
            print(f"NEW GOAL POS: {goal_position}")
            print(f"NEW GOAL ORI: {goal_orientation}")

        # State transition
        #state = State.CATCH
        # DON'T TRANSITION, BC WANT TO KEEP READING. Instead, update prior to check for change
        last_ball_position = current_ball_position
        last_ball_velocity = current_ball_velocity

    # elif state == State.CATCH:

    #     # Just continually command catch position
    #     redis_client.set(redis_keys.catching_position, json.dumps(goal_position.tolist()))
    #     redis_client.set(redis_keys.catching_orientation, json.dumps(goal_orientation.tolist()))

except KeyboardInterrupt:
  print("Keyboard interrupt")
  pass
except Exception as e:
  print(e)
  pass