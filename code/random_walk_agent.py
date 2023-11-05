import time
import gym
import numpy as np
import socnavgym
import os
import pygame
import numpy as np 
import sys
import argparse
import time
import json
from glob import glob
from pathlib import Path
import random


os.environ['PYQTGRAPH_QT_LIB'] = 'PySide2'
from PySide2 import QtWidgets
import pyqtgraph as pg

ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num_episodes", required=False, default=3, help="number of episodes")
ap.add_argument("-c", "--config", required=False, default="code/env_config.yaml", help="Environment config file")
ap.add_argument("-r", "--record", required=False, default=True, help="Whether you want to record the observations, and actions or not")
ap.add_argument("-s", "--start", required=False, default=0, help="starting episode number")
args = vars(ap.parse_args())
episodes = int(args["num_episodes"])

pygame.init()


app = QtWidgets.QApplication(sys.argv)


start = int(args["start"])
if os.path.isdir("./episode_recordings/"):
    for path in Path("./episode_recordings/").rglob('*.json'):
        path = str(path)
        start = max(int(path.split("/")[-1].split(".")[0]), start)
env = gym.make("SocNavGym-v1", config=args["config"], disable_env_checker=True)

total_sums = []
import time

for episode in range(episodes):
    env.reset()
    done = False

    step = -1
    prev_sum = 0
    x = []
    sums = []
    rewards = []
    sngnn = []
    total_reward = 0
    episode_list = []
    first_timestamp = time.time()

    while not done:
        step += 1

        vel_x = random.random()
        vel_y = random.random()
        vel_a = random.random()-0.5

        if env.robot.type == "diff-drive": vel_y = 0


        obs, rew, terminated, truncated, info = env.step([vel_x, vel_y, vel_a])
        obs["action"] = np.array([vel_x, vel_y, vel_a], dtype=np.float32)
        obs["reward"] = np.array([rew], dtype=np.float32)
        for key in obs.keys():
            if obs[key] is None:
                # print(f"{key=} is none")
                pass
            else:
                # print(f"{key=} is {obs[key].shape}")
                obs[key] = obs[key].tolist()
        obs['timestamp'] = time.time()-first_timestamp
        episode_list.append(obs)
        done = terminated or truncated

        app.processEvents()
        env.render()

        env.record("./episode_recordings/" + str(episode+1+start).zfill(8) + ".mp4")

        # time.sleep(1000)
        if done:
            print(f"Total reward : {total_reward}")
            env.reset()
            if args["record"]:
                if not os.path.isdir("./episode_recordings/"):
                    os.makedirs("./episode_recordings/")
                with open("./episode_recordings/" + str(episode+1+start).zfill(8) + ".json", "w") as f:
                    json.dump(episode_list, f, indent=2)
            
        app.processEvents()

    
