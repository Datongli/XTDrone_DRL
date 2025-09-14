#!/home/ldt/anaconda3/envs/deeplearning/bin/python
# -*- coding: utf-8 -*-
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
import hydra
# import wandb
import time
import numpy as np
from env.scripts.env import StaticObstacleEnv


FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="train", version_base=None)
def main(cfg):
    env = StaticObstacleEnv(cfg)
    time.sleep(2)
    state =  env.reset()  # 测试环境的reset功能
    """测试环境的step功能"""
    actions = np.array([[[-1, 0, 0, np.pi/2], [1, 0, 0, np.pi/2]]])
    while True:
        while True:
            for action in actions:
                nextState, reward = env.step(action)
            # 先用两个做测试
            if env.uavs[0].done and env.uavs[1].done:
                break
        env.reset()


if __name__ == "__main__":
    main()