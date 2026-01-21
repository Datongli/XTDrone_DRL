#!/home/ldt/anaconda3/envs/deeplearning/bin/python
# -*- coding: utf-8 -*-
import sys
import os
# 若当前解释器不是指定的 conda 解释器，则切换到它重新执行脚本
CONDA_PY = "/home/ldt/anaconda3/envs/deeplearning/bin/python"
if os.path.exists(CONDA_PY) and os.path.realpath(sys.executable) != os.path.realpath(CONDA_PY):
    os.execv(CONDA_PY, [CONDA_PY, os.path.abspath(__file__), *sys.argv[1:]])
# 让包导入与 train.py 一致
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
import hydra
from tqdm import tqdm
import signal
import time
import numpy as np
import torch
from env.scripts.ego_planner_env import EGOPlannerEnv
from uav.scripts.uav import UAVInfo
from hydra.utils import to_absolute_path
from train.scripts.tools import cfg_get, to_plain_cfg, load_checkPoint
from train.scripts.tools import set_gazebo_unlimit, kill_gzclient


# 复用 train 的 cfg 目录
FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "train/cfg")
@hydra.main(config_path=FILE_PATH, config_name="test", version_base=None)
def test(cfg) -> None:
    """
    测试脚本
    在自己的环境上测试ego-planner算法的效果
    :param cfg: 配置文件
    :return: None
    """
    """创建环境"""
    print("测试ego-planner算法效果")
    # 初始化环境
    env = EGOPlannerEnv(cfg, mode="test")
    # 是否关闭gazebo的gui页面来加速
    if not bool(cfg_get(cfg, "gazebo.gui", False)):
        # 关闭gazebo headless客户端，防止渲染占用
        kill_gzclient()
    time.sleep(2)
    # Gazebo 加速（可选）
    turboEnabled = bool(cfg_get(cfg, "gazebo.turbo.enabled", True))  # 是否启用加速
    gazeboTurboApplied = False
    """捕获 Ctrl+C 以安全退出"""
    stopRequested = {"flag": False}
    def handle_sigint(signum, frame):
        stopRequested["flag"] = True
        print("\n[测试] 收到中断信号，将安全退出...")
    signal.signal(signal.SIGINT, handle_sigint)
    """获取测试参数"""
    totalEpisodes = int(cfg_get(cfg, "eval.numEpisodes", 1))
    # 全局统计
    totalSuccess, totalCollision, totalOver = 0, 0, 0
    returns = []
    """进行测试"""
    with tqdm(total=totalEpisodes, desc="[Testing]") as progressBar:
        for episode in range(totalEpisodes):
            # 应用 gazebo 加速一次
            if turboEnabled and not gazeboTurboApplied:
                set_gazebo_unlimit(
                    maxWaitTime=20,
                    timeStep=None,
                    targeModels=None
                )
                gazeboTurboApplied = True
            # 环境重置
            states = env.reset()
            time.sleep(2.0)


            # 先丑陋的把reset跑通，观察效果
            while True:
                env.step()




if __name__ == "__main__":
    test()