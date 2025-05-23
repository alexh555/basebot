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

from filterpy.kalman import KalmanFilter

DEG_TO_RAD = math.pi / 180.0
NUM_POINTS_EST = 3
CUTOFF_INITIAL = 3.5

STOP_CUTOFF = 0.25 # Where it starts assuming ball has been caught

init_position = np.array([0.1, -0.50, 0.5]) # Where to put net center at start
init_orientation = np.array([[1.0,0,0],[0,-1.0,0],[0,0,-1.0]])

estimated_init_velo = np.array([-5.0, 0.0, 1.0])
kf_init_estimate = np.array([CUTOFF_INITIAL, 0.0, 1.0, -5.0, 0.0, 1.0])

class State(Enum):
  INIT = auto()
  FINDING = auto()
  CATCH = auto()
  FINISH = auto()

""" HELPER FUNCTIONS """
def transform_ball2arm(ball_pos, arm_pos):
   
   """
   Transform ball coordinates to arm origin
   """

   # Flip axes appropriately
   arm_pos = list(arm_pos)
   orig_arm_pos = arm_pos.copy()
   arm_pos[0] = -orig_arm_pos[1]
   arm_pos[1] = orig_arm_pos[0]
   arm_pos[2] = orig_arm_pos[2]
   arm_pos = np.array(arm_pos)

   ball_pos = list(ball_pos)
   orig_ball_pos = ball_pos.copy()
   ball_pos[0] = -orig_ball_pos[1]
   ball_pos[1] = orig_ball_pos[0]
   ball_pos[2] = orig_ball_pos[2]
   ball_pos = np.array(ball_pos)

   # Perform subtraction
   # R(arm in world) + R(ball in arm) = R(ball in world)
   # therefore
   # R(ball in arm) = R(ball in world) - R(arm in world)

   ball_in_arm = ball_pos - arm_pos
   return ball_in_arm

def transform_Optitrack2Sim(position):
    """
    Transform Optitrack room coords to those making sense in simulator
    """

    # FIT TO SIM - ORIGINAL
    # position = list(position)
    # orig_pos = position.copy()
    # position[0] = orig_pos[2] + 5 - 0.5 # Remove second term to to align with blue X
    # position[1] = orig_pos[0] - 0.75 #- 0.7 # Remove second term to to align with blue X
    # position[2] = orig_pos[1] - 0.4
    # position = np.array(position)

    # FIT TO SIM - Someone redefined the origin
    position = list(position)
    orig_pos = position.copy()
    position[0] = -orig_pos[1] + 5 - 0.5 # Remove second term to to align with blue X
    position[1] = orig_pos[0] #- 0.7 # Remove second term to to align with blue X
    position[2] = orig_pos[2] - 0.4
    position = np.array(position)
   
    return position
   

def velocity_polyfit(times, positions, t_eval=None):
    """
    Fits a 2nd-order polynomial to position vs time, returns velocity at t_eval.

    times: list/array of timestamps (length N)
    positions: array of shape (N, 3)
    t_eval: time at which to evaluate velocity (default: latest time)
    """
    times = np.array(times)
    positions = np.array(positions)
    
    if t_eval is None:
        t_eval = times[-1]  # most recent time

    velocity = np.zeros(3)
    for i in range(3):  # for x, y, z
        coeffs = np.polyfit(times, positions[:, i], deg=2)
        a, b, _ = coeffs
        velocity[i] = 2 * a * t_eval + b

    return velocity

# def position_polyfit(times, positions, x_cut = 0.1)
#    """
#    Position prediction based on polynomail fitting
#    """

#    times = np.array(times)
#    positions = np.array(positions)

def velocity_kf(delta_t):
   kf = KalmanFilter(dim_x=6, dim_z=3)

   # State matrix (position, velocity)
   kf.F = np.block([
      [np.eye(3), dt* np.eye(3)],
      [np.zeros((3,3)), np.eye(3)]
   ])

   # Measurement function (position only)
   kf.H = np.hstack([np.eye(3), np.zeros((3,3))])

   # Initial state estimate
   #kf.x = np.zeros((6,1))
   kf.x = kf_init_estimate

   # Covariances (TO BE TUNED)
   kf.P *= 1.0
   kf.R *= 0.0001
   kf.Q = np.block([
      [1e-6 * np.eye(3),      np.zeros((3, 3))],
      [np.zeros((3, 3)),  1e-3 * np.eye(3)]     # ← velocity noise: allow adaptation
    ])

   return kf

def estimate_velocity_cd(pos, delta_t):
    """
    Function for estimating velocity from 3 position measurements using a central difference
    """

    #velo_est = (-3*pos[0] + 32*pos[1] - 168*pos[2] + 672*pos[3] - 672*pos[5] + 168*pos[6] - 32*pos[7] + 3*pos[8])/(280*delta_t)
    #velo_est = (3*pos[2] - 4*pos[1] + pos[0])/(2*delta_t)
    velo_est = (25*pos[4] - 48*pos[3] + 36*pos[2] - 16*pos[1] + 3*pos[0])/(12*delta_t)

    return velo_est

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
    #print(f"Final catch pos = {current_position}")

    # Clip at limits
    lower_pos_bound = np.array([-0.1, -0.9, 0.2])
    upper_pos_bound = np.array([0.25, -0.1, 1.1 ])

    current_position = np.clip(current_position, lower_pos_bound, upper_pos_bound)
    #next_ori = velo_to_ori(current_velocity)
    next_ori = init_orientation # For now, maintain orientation to speed things up and avoid weird motions
    
    print("") #Debugger
    print(f"~~~New velo and goal:{current_velocity}, {current_position}")

    return current_position, next_ori, True

    # DON'T MOVE at limits
    # if ( abs(current_position[0]) < 0.25 and
    #         current_position[1] < -0.1 and
    #         current_position[1] > -0.9 and
    #         current_position[2] > 0.1 and
    #         current_position[2] < 1.1 ):

    #     next_ori = velo_to_ori(current_velocity)

    #     #Debugger
    #     print("")
    #     print(f"~~~New velo and goal:{current_velocity}, {current_position}")

    #     return current_position, next_ori, True
    # else:
    #     #print("!!!ERROR!!! Final catchable pos not reachable")
    #     return init_position, init_orientation, False # Maintain starting position, defined below
  

def velo_to_ori(velocity_vec):
    """
    Computes robot orientation given catch point velocity of ball
    """

    # Safeguard against divide by 0
    if (np.linalg.norm(velocity_vec) > 1e-6):
       
        x_des = -velocity_vec / np.linalg.norm(velocity_vec) # Desired x axis should be anti-parallel to velocity
    else:
       x_des = np.array([1.0, 0.0, 0.0]) # Default to initial orientation

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

""" End HELPER FUNCTIONS """

# CHANGE TO TRUE TO RUN IN SIMULATION
simulation = True

# *** REDIS SETUP ***
RIGID_BODY_POS_KEY = "sai2::optitrack::rigid_body_pos::"
BALL_RB_NUM = "4" # Change to match Motive
ARM_RB_NUM = "5"

N_ESTIMATE = 3 # Number of points needed to 

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
  ball_position: str = RIGID_BODY_POS_KEY + BALL_RB_NUM
  arm_position: str = RIGID_BODY_POS_KEY + ARM_RB_NUM

  catching_position: str = "sai::sim::panda::catching::position"
  catching_orientation: str = "sai::sim::panda::catching::orientation"


redis_keys = RedisKeys()

# redis client
redis_client = redis.Redis()
   
# loop at 100 Hz
loop_time = 0.0
dt = 0.01
internal_step = 0
state = State.INIT

time.sleep(0.01)
init_time = time.perf_counter_ns() * 1e-9

time_start = 0


# *** POSITION INITIALIZATION ***
goal_position = init_position # Set as where to start
goal_orientation = init_orientation

current_ball_position = np.array([0.0, 0.0, 0.0])
current_ball_velocity = np.array([0.0, 0.0, 0.0])

tracking_list = [] # Empty lists for storing ball positions and times 
time_list = []

pos_error = 1
ori_error = 1

velKF = velocity_kf(dt)

baseline_ball_position = np.array([5.0, 0.0, 1.0])

# Check dt computation
t_prev = time.time()
t_next = time.time()
pos_prev = 0.0
pos_next = 0.0
cur_dt = 0.0

firstFlag = True
first_time = 0.0

try:
  print("...Starting...")

  # Get arm position
  if( redis_client.exists(redis_keys.arm_position) ):
        armpos_str = redis_client.get(redis_keys.arm_position).decode("utf-8")
        arm_static_pos = np.array([float(p) for p in armpos_str.strip("[]").split(",")])  # Initial position (m)

  while True:
    loop_time += dt
    time.sleep(max(0, loop_time - (time.perf_counter_ns() * 1e-9 - init_time)))
    
    # *** UPDATE ROBOT STATE ***
    # read robot state
    if( redis_client.exists(redis_keys.cartesian_task_current_position) and redis_client.exists(redis_keys.cartesian_task_current_orientation) ):
        current_position = np.array(json.loads(redis_client.get(redis_keys.cartesian_task_current_position)))
        current_orientation = np.array(json.loads(redis_client.get(redis_keys.cartesian_task_current_orientation)))
    else:
       print("ERROR: no robot pos posted")

    pos_error = np.linalg.norm(goal_position - current_position)
    ori_error = np.linalg.norm(goal_orientation - current_orientation)

    # state machine
    if state == State.INIT:

        # Ensure got to intial position
        #if pos_error < 1e-2 and ori_error < 1e-1:
        goal_position = init_position
        goal_orientation = init_orientation
        print("Moved to INIT position")

        #redis_client.delete(redis_keys.ball_position) # Clear the ball redis key, so it doesn't have old run info
        redis_client.set(redis_keys.ball_position, json.dumps(baseline_ball_position.tolist())) # Set to clean baseline value
        print("~Reset ball redis key~")

        state = State.FINDING # If so, look for ball


    elif state == State.FINDING:

       # Now, check for ball info is available
        if( redis_client.exists(redis_keys.ball_position) ):

            # Read in init ball 
            bpos_str = redis_client.get(redis_keys.ball_position).decode("utf-8")
            orig_ball_pos = np.array([float(p) for p in bpos_str.strip("[]").split(",")])  # Initial position (m)
            
            current_ball_position = transform_ball2arm(orig_ball_pos, arm_static_pos)
                      
            # CHECK INITIAL POS CUTOFF
            if (current_ball_position[0] < CUTOFF_INITIAL):

                # Check if ball past catchable position
                if (current_ball_position[0] < STOP_CUTOFF):
                    state = State.FINISH # Stop updating, just move to catch position and stop

                    tracking_list = [] # Clear polynomial lists
                    time_list = []
                else:

                    """ KALMAN APPROACH """
                    # Kalman update
                    if firstFlag:
                        # Hand first reading
                        t_prev = time.time()
                        pos_prev = current_ball_position
                        firstFlag = False

                        initial_state_estimate = np.concatenate(current_ball_position, estimated_init_velo)
                        velKF.x = np.array(initial_state_estimate)
                    else:
                        t_cur = time.time()
                        
                        if (np.linalg.norm(current_ball_position - pos_prev) > 1e-6): # Only if pos is new
                
                            # Predict with current dt
                            dt_cur = t_cur - t_prev
                            velKF.F = np.block([
                                            [np.eye(3), dt_cur* np.eye(3)],
                                            [np.zeros((3,3)), np.eye(3)]
                                        ])                          
                            z = np.array(current_ball_position).reshape((3,1))
                            velKF.predict()
                            velKF.update(z)

                            current_ball_velocity = velKF.x[3:].flatten()
                            #print(f"VELOCITY ESTIMATE = {current_ball_velocity}")
                                
                            # Generate resulting catch point
                            new_position, new_orientation, new_flag = get_catch_point(current_ball_position, current_ball_velocity)
                            goal_position = new_position
                            goal_orientation = new_orientation
                            #print(f"NEW GOAL POS: {goal_position}")

                            # Update previous
                            t_prev = t_cur
                            pos_prev = current_ball_position

                    """ VELOCITY APPROACH """
                    # if firstFlag:
                    #    # Hand first reading
                    #    t_prev = time.time()
                    #    pos_prev = current_ball_position
                    #    firstFlag = False
                    # else:
                    
                    #    # Update velocity estimate
                    #    t_next = time.time()
                    #    cur_dt = t_next - t_prev

                    #    current_ball_velocity = (current_ball_position - pos_prev)/(cur_dt)
                    #    #print(f"VELOCITY ESTIMATE = {current_ball_velocity}")

                    #    # Generate resulting catch point
                    #    # ONLY GET CATCH POINT AND UPDATE PREVIOUS IF NON-ZERO VELOCITY (diff in pos)
                    #    if (np.linalg.norm(current_ball_velocity) > 1e-6):
                    #       new_position, new_orientation, new_flag = get_catch_point(current_ball_position, current_ball_velocity)
                          
                    #       goal_position = new_position
                    #       goal_orientation = new_orientation                          
                          
                    #       # Update for next estimate
                    #       t_prev = t_next
                    #       pos_prev = current_ball_position
                    
                    """ POLYFIT APPROACH """
                    # if firstFlag:
                    #    # Gracefully handle first
                    #    first_time = time.time()
                    #    time_list.append(first_time)
                    #    tracking_list.append(current_ball_position)
                    #    firstFlag = False
                    # else:
                    #     next_time = time.time()
                    #     time_list.append(next_time - first_time)
                    #     tracking_list.append(current_ball_position)

                    # if(len(tracking_list) > NUM_POINTS_EST):
                    #     current_ball_velocity = velocity_polyfit(time_list, tracking_list, time.time())
                    #     print(f"VELOCITY ESTIMATE = {current_ball_velocity}")

                    #     # Generate resulting catch point
                    #     new_position, new_orientation, new_flag = get_catch_point(current_ball_position, current_ball_velocity)
                    #     #if new_flag: # Only update goal position on valid re-estimate
                    #     goal_position = new_position
                    #     goal_orientation = new_orientation
                    #     #print(f"NEW GOAL POS: {goal_position}")
                    
                    """ SINGLE ESTIMATE APPRAOCH """
                    # if (len(tracking_list) < NUM_POINTS_EST):
                    #     tracking_list.append(current_ball_position)

                    #     # Time check
                    #     if (len(tracking_list) == 1):
                    #     t_prev = time.time()
                    #     else:
                    #     t_next = time.time()
                    #     cur_dt = t_next - t_prev
                    #     print(f"cur_dt = {cur_dt}")
                    #     t_prev = t_next
                    # else:

                    #     # Time check
                    #     t_next = time.time()
                    #     cur_dt = t_next - t_prev
                    #     print(f"cur_dt = {cur_dt}")

                    #     # Estimate velocity from position
                    #     #current_ball_velocity = estimate_velocity_cd(tracking_list, dt)
                    #     current_ball_velocity = estimate_velocity_cd(tracking_list, dt)
                    #     print(f"VELOCITY ESTIMATE = {current_ball_velocity}")
                        
                    #     # Generate resulting catch point
                    #     new_position, new_orientation = get_catch_point(current_ball_position, current_ball_velocity)
                    #     goal_position = new_position
                    #     goal_orientation = new_orientation
                    #     print(f"NEW GOAL POS: {goal_position}")
                    #     print(f"NEW GOAL ORI: {goal_orientation}")
                    #     state = State.CATCH # State transition

                    #     # Only move to catch if it is diff than initial
                    #     # delta_pos = new_position - init_position
                    #     # if ( np.linalg.norm(delta_pos) > 1e-6):
                    #     #     goal_position = new_position
                    #     #     goal_orientation = new_orientation

                    #     #     print(f"NEW GOAL POS: {goal_position}")
                    #     #     print(f"NEW GOAL ORI: {goal_orientation}")
                            
                    #     #     # State transition
                    #     #     state = State.CATCH
            else:
               print(f"Ball BEFORE line at: {current_ball_position}")
       

    elif state == State.CATCH:
        # Check if got to goal catching position
        if pos_error < 1e-2 and ori_error < 1e-1:
           state = State.FINISH # If so, stop
    
    elif state == State.FINISH:

        # Wait until ball returns to start, then return to finish
        print("------Waiting for reset")
        if( redis_client.exists(redis_keys.ball_position) ):

            # Read in init ball 
            bpos_str = redis_client.get(redis_keys.ball_position).decode("utf-8")
            orig_ball_pos = np.array([float(p) for p in bpos_str.strip("[]").split(",")])  # Initial position (m)
            
            current_ball_position = transform_ball2arm(orig_ball_pos, arm_static_pos)

            if (current_ball_position[0] > CUTOFF_INITIAL):
                print("---RESETTING---")
                goal_position = init_position
                goal_orientation = init_orientation

                # Reset Kalman
                velKF.x = kf_init_estimate
                velKF.P = np.eye(6)

                state = State.INIT

        # Go back to initial
        # input("Press enter to reset: ") 
        # goal_position = init_position
        # goal_orientation = init_orientation

        # state = State.INIT


    # *** ALWAYS TRANSMIT GOAL, unless at end ***
    if state != State.FINISH:
        redis_client.set(redis_keys.catching_position, json.dumps(goal_position.tolist()))
        redis_client.set(redis_keys.catching_orientation, json.dumps(goal_orientation.tolist()))

except KeyboardInterrupt:
  print("Keyboard interrupt")
  pass
except Exception as e:
  print(e)
  pass