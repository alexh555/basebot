# simulate optitrck data for throwing a ball
import numpy as np
import redis
import json
import time
from dataclasses import dataclass


@dataclass
class RedisKeys:
    ball_pos: str = "sai::sim::ball::sensors::position"
    ball_vel: str = "sai::sim::ball::sensors::velocity"
  

init_pos = np.array([5.5, 0.0, 1.0])
init_vel = np.array([-8.0, 0.0, 2.0])


redis_keys = RedisKeys()
redis_client = redis.Redis()


def throw_ball(init_pos, init_vel):
    dt = 0.01
    t = 0.0
    ball_pos = init_pos
    ball_vel = init_vel
    while ball_pos[2] >= 0.0:
        print(f"t: {t:.2f}, ball_pos: {ball_pos}, ball_vel: {ball_vel}")
        ball_pos = ball_pos + ball_vel * dt
        ball_vel = ball_vel + np.array([0.0, 0.0, -9.81]) * dt
        t += dt
        # push data to redis
        redis_client.set(redis_keys.ball_pos, '[' + ', '.join(map(str, ball_pos)) + ']')
        redis_client.set(redis_keys.ball_vel, '[' + ', '.join(map(str, ball_vel)) + ']')
        time.sleep(dt)

throw_ball(init_pos, init_vel)