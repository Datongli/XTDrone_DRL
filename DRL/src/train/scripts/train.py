#!/home/ldt/anaconda3/envs/deeplearning/bin/python
# -*- coding: utf-8 -*-
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
import hydra
import wandb
import time
from env.scripts.env import StaticObstacleEnv


FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="train", version_base=None)
def main(cfg):
    env = StaticObstacleEnv(cfg)
    time.sleep(2)
    env.reset()  # 测试环境的reset功能
    while True:
        time.sleep(10)
        env.reset()


if __name__ == "__main__":
    main()