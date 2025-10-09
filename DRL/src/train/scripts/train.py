#!/home/ldt/anaconda3/envs/deeplearning/bin/python
# -*- coding: utf-8 -*-
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
import hydra
import wandb
from tqdm import tqdm
import time
import numpy as np
from env.scripts.env import StaticObstacleEnv
from navigation.scripts.SAC import SAC
from uav.scripts.uav import UAVInfo
from tools import ReplayBuffer, moving_average, epsilon_annealing




# FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "cfg")
# @hydra.main(config_path=FILE_PATH, config_name="train", version_base=None)
# def main(cfg) -> None:
#     """
#     训练的主函数
#     :param cfg: 配置
#     :return: None
#     """
#     env = StaticObstacleEnv(cfg)
#     time.sleep(2)
#     state =  env.reset()  # 测试环境的reset功能
#     """测试环境的step功能"""
#     actions = np.array([[[-1, 0, 0, np.pi/2], [1, 0, 0, np.pi/2]]])
#     while True:
#         while True:
#             for action in actions:
#                 nextState, reward = env.step(action)
#             # 先用两个做测试
#             if env.uavs[0].done and env.uavs[1].done:
#                 break
#         env.reset()



FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="train", version_base=None)
def main(cfg) -> None:
    """
    训练的主函数
    :param cfg: 配置
    :return: None
    """
    """创建环境"""
    navigationAlgorithm = SAC(cfg)  # 实例化路径规划算法
    replayBuffer = ReplayBuffer(cfg.bufferSize)  # 实例化经验回放缓冲区
    env = StaticObstacleEnv(cfg)  # 实例化环境
    time.sleep(2)
    states =  env.reset()  # 获取状态信息（字典）
    firstFlag = True
    """训练迭代"""
    for j in range(10):
        with tqdm(total=int(cfg.numEpisodes / 10), desc="Training %d" % j) as pbar:
            for iEpisode in range(int(cfg.numEpisodes / 10)):
                """初始化一些记录变量"""
                successCount = 0  # 成功到达目标点的无人机个数
                episodeReturn = 0  # 迭代的奖励总和
                collisionCount = 0  # 发生碰撞的无人机个数
                overCount = 0  # 超出最大步长的无人机个数
                """"环境重置"""
                if firstFlag:
                    firstFlag = False
                else:
                    states = env.reset()
                time.sleep(2)
                doneCount = 0  
                """进行每一步的动作"""
                while doneCount < cfg.uav.uavNums:
                    doneCount = 0  # 达到终止状态的无人机个数（包括达到目标点、发生碰撞、超出最大步长）
                    states = [state for state in states if state != None]  # 删除None状态
                    actions = navigationAlgorithm.take_action(states)  # 根据状态获取动作
                    actions = np.array(actions)  # 转为numpy数组
                    # 获取下一步的状态，奖励，是否完成
                    nextStates, rewards, dones = env.step(actions)
                    """存储数据"""
                    for reward in rewards:
                        if reward != None:
                            episodeReturn += reward
                    # 计数器
                    statusCounters = {
                        UAVInfo.SUCCESS: 0,
                        UAVInfo.COLLISION: 0,
                        UAVInfo.STEP_OVER: 0,
                        UAVInfo.NORMAL: 0,
                    }
                    for uav in env.uavs:
                        statusCounters[uav.info] += 1
                        if uav.done:
                            doneCount += 1
                    # 统计数据
                    successCount = statusCounters[UAVInfo.SUCCESS]
                    collisionCount = statusCounters[UAVInfo.COLLISION]
                    overCount = statusCounters[UAVInfo.STEP_OVER]
                    """将数据存入经验回放缓冲区"""
                    i = 0
                    # 根据state是否为None来判断是否完成
                    for state in states:
                        if state == None or nextStates[i] == None:
                            continue
                        else:
                            replayBuffer.add(state, actions[i], rewards[i], nextStates[i], dones[i])
                        i += 1
                    states = nextStates  # 更新状态
                """迭代一次，更新网络"""
                if replayBuffer.size() > cfg.minimalSize:
                    batchStates, batchActions, batchRewards, batchNextStates, batchDones = replayBuffer.sample(cfg.batchSize)
                    navigationAlgorithm.update({"states": batchStates,
                                                "actions": batchActions,
                                                "rewards": batchRewards,
                                                "nextStates": batchNextStates,
                                                "dones": batchDones})
                    # 学习率更新
                    navigationAlgorithm.actorScheduler.step()
                    navigationAlgorithm.criticScheduler1.step()
                    navigationAlgorithm.criticScheduler2.step()
                """打印每一步的训练信息"""
                pbar.set_postfix({
                    "episode":
                    "%d" % (iEpisode + j * int(cfg.numEpisodes / 10)),
                    "return": "%.3f" % episodeReturn,
                    "successCount": "%d" % successCount,
                    "collisionCount": "%d" % collisionCount,
                    "overCount": "%d" % overCount
                })
                pbar.update(1)



if __name__ == "__main__":
    main()