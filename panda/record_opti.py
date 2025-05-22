import time
import redis
import sys

#2 minutes max per file 
MAX_RUN_TIME = 120  

BALL_POS_KEY = "sai::sim::ball::initial::position"
BALL_VEL_KEY = "sai::sim::ball::initial::velocity"


if len(sys.argv) != 2:
    print("Input error. Use python record_opt.py <output_file>")
    exit(1)

save_path = sys.argv[1]

redis_client = redis.Redis()

with open(save_path, "w") as file:
    file.write("time, ball_pos, ball_vel\n")
    start_time = time.time()

    while time.time() - start_time < MAX_RUN_TIME:
        timestamp = time.time()
        ball_pos = redis_client.get(BALL_POS_KEY).decode("utf-8")
        ball_vel = redis_client.get(BALL_VEL_KEY).decode("utf-8")
        file.write(f"{timestamp}, {ball_pos}, {ball_vel}\n")
