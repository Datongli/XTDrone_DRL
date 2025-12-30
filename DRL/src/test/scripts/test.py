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
from env.scripts.env import StaticObstacleEnv
from navigation.scripts.SAC import SAC
from navigation.scripts.DDPG import DDPG
from navigation.scripts.MTransSAC import MTransSAC
from uav.scripts.uav import UAVInfo
from hydra.utils import to_absolute_path
from train.scripts.tools import cfg_get, to_plain_cfg, load_checkPoint
from train.scripts.tools import set_gazebo_unlimit, kill_gzclient
from tools import load_pt_models, load_pth_models, switch_model_eval_MTransSAC, switch_model_eval_DDPG


# 复用 train 的 cfg 目录
FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "train/cfg")
@hydra.main(config_path=FILE_PATH, config_name="test", version_base=None)
def test(cfg) -> None:
    """
    测试脚本
    加载训练好的模型并测试
    在环境中运行若干回合，统计成功/碰撞/超步长等指标
    :param cfg: 配置文件
    :return: None
    """
    # 注册表
    ALGORITHM_REGISTRY = {
        "DDPG": DDPG(cfg),
        "SAC": SAC(cfg),
        "MTrans-SAC": MTransSAC(cfg)
    }
    """创建环境"""
    # 根据配置选择算法
    algorithmType = cfg_get(cfg, "eval.navigationModel", "SAC")
    navigationAlgorithm = ALGORITHM_REGISTRY.get(algorithmType, None)
    if navigationAlgorithm is None:
        raise ValueError(f"[测试] 未知算法类型: {algorithmType}")
    print(f"[测试] 使用算法: {algorithmType}")
    env = StaticObstacleEnv(cfg)  # 导入环境
    if not bool(cfg_get(cfg, "gazebo.gui", False)):
        # 关闭gazebo headless客户端，防止渲染占用
        kill_gzclient()
    time.sleep(2)
    # Gazebo 加速（可选）
    turboEnabled = bool(cfg_get(cfg, "gazebo.turbo.enabled", True))  # 是否启用加速
    gazeboTurboApplied = False
    """加载需要测试的模型"""
    # 加载checkPoint
    checkPointDirCfg = cfg_get(cfg, "wandb.checkPointDir", "./checkPoints")
    checkPointDir = to_absolute_path(os.path.expanduser(checkPointDirCfg))
    latestCheckPointPath = os.path.join(checkPointDir, "ep_199.pt")
    # 允许从 cfg 指定测试用 checkpoint
    preferPath = cfg_get(cfg, "eval.checkPointPath", None)
    if preferPath:
        preferPath = to_absolute_path(os.path.expanduser(preferPath))
    # 判断加载模式
    loadSuccess = False
    if algorithmType == "DDPG":
        searchDir = preferPath if preferPath else checkPointDir
        print(f"[测试] 尝试从目录加载 .pth 文件: {searchDir}")
        loadSuccess = load_pth_models(searchDir, navigationAlgorithm, cfg_get(cfg, "device", "cpu"))
    elif algorithmType == "MTrans-SAC":
        # MTrans-SAC 目前仅支持 .pt 格式
        ptName = "SAC_only_mean_good.pt"
        loadCheckPointPath = os.path.join(preferPath if preferPath else checkPointDir, ptName)  # 要加载的检查点路径
        # 加载模型参数
        loadSuccess = load_pt_models(loadCheckPointPath, navigationAlgorithm)
    if not loadSuccess:
        latestCheckPointPath = os.path.join(checkPointDir, "latest.pt")
        loadFromPath = preferPath if (preferPath and os.path.exists(preferPath)) else latestCheckPointPath
        if loadFromPath and os.path.exists(loadFromPath):
            print(f"[测试] 尝试加载 .pt 文件: {loadFromPath}")
            try:
                # 验证文件完整性
                fileSize = os.path.getsize(loadFromPath)
                if fileSize < 1024:
                    print(f"[测试] checkpoint 文件太小 ({fileSize} bytes)，可能损坏")
                else:
                    print(f"[测试] checkpoint 文件大小: {fileSize / 1024 / 1024:.2f} MB")
                    # load_checkPoint 返回 (startEpisodeIndex, replayBuffer)
                    _, _ = load_checkPoint(loadFromPath, navigationAlgorithm, replayBuffer=None)
                    print(f"[测试] ✓ 已加载 .pt 模型参数: {loadFromPath}")
                    loadSuccess = True
            except RuntimeError as e:
                if "failed finding central directory" in str(e):
                    print(f"[测试] checkpoint 文件损坏（zip格式错误）")
                else:
                    print(f"[测试] 加载 .pt 模型失败: {e}")
            except Exception as e:
                print(f"[测试] 加载 .pt 模型失败: {e}")
    # 加载失败则退出
    if not loadSuccess:
        print("[测试] 未能加载任何模型，退出测试")
        return
    """开启模型的评估模式"""
    try:
        if algorithmType == "MTrans-SAC":
            switch_model_eval_MTransSAC(navigationAlgorithm)
        elif algorithmType == "DDPG":
            switch_model_eval_DDPG(navigationAlgorithm)
        print("[测试] 模型已切换到评估模式")
    except Exception as e:
        print(f"[测试] 加载模型失败：{e}")
        return
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
    # 不进行梯度追踪
    with torch.no_grad():
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
                # 该回合统计值
                episodeReturn = 0.0  # 该回合累计奖励
                doneCount = 0  # 完成的无人机个数
                successCount = 0  # 成功降落的无人机个数
                collisionCount = 0  # 碰撞的无人机个数
                overCount = 0  # 超步长的无人机个数
                landedCount = 0  # 着陆的无人机个数
                while doneCount < cfg.uav.uavNums:
                    doneCount = 0
                    # 获得状态
                    states = [s for s in states if s is not None]
                    # 选择动作
                    # actions = navigationAlgorithm.take_action(states)
                    # actions = np.array(actions, dtype=np.float32)
                    # 选择动作（跳过正在降落的无人机）
                    activeStates = []
                    activeIndices = []
                    # 构造Batch
                    batchUavStates = []
                    batchSensorStates = []
                    for i, uav in enumerate(env.uavs):
                        # 跳过降落中和已完成的无人机
                        if not uav.done and not uav.isLanding:
                            if i < len(states):
                                activeStates.append(states[i])
                                activeIndices.append(i)
                                uavData = states[uav.uavID]  # 获取当前无人机的状态
                                batchUavStates.append(uavData["uavState"])
                                batchSensorStates.append(uavData["sensorData"])
                    # 堆叠数据
                    batchInput = {
                        "uavState": np.stack(batchUavStates),
                        "sensorState": np.stack(batchSensorStates)
                    }
                    # 构建动作数组
                    actions = np.zeros((cfg.uav.uavNums, 4), dtype=np.float32)
                    if activeStates:
                        if algorithmType == "MTrans-SAC":
                            activeActions, _ = navigationAlgorithm.take_action(batchInput, deterministic=True)
                            # 给每个动作末尾追加一个常数 np.pi/2
                            activeActions = np.asarray(activeActions, dtype=np.float32)
                            activeActions = np.concatenate(
                                [activeActions, np.full((activeActions.shape[0], 1), np.pi / 2, dtype=np.float32)],
                                axis=1
                            )
                        else:
                            activeActions = navigationAlgorithm.take_action(activeStates)
                        for idx, activeIdx in enumerate(activeIndices):
                            actions[activeIdx] = activeActions[idx]
                    # 执行一步
                    nextStates, rewards, dones = env.step(actions)
                    # 累计回报
                    for reward in rewards:
                        if reward is not None:
                            episodeReturn += float(reward)
                    # 统计UAV状态
                    statusCounters = {
                        UAVInfo.SUCCESS: 0,
                        UAVInfo.COLLISION: 0,
                        UAVInfo.STEP_OVER: 0,
                        UAVInfo.NORMAL: 0,
                        UAVInfo.LANDED: 0,
                        UAVInfo.LANDING: 0
                    }
                    for uav in env.uavs:
                        statusCounters[uav.info] += 1
                        if uav.done:
                            doneCount += 1
                    successCount = statusCounters[UAVInfo.SUCCESS]
                    collisionCount = statusCounters[UAVInfo.COLLISION]
                    overCount = statusCounters[UAVInfo.STEP_OVER]
                    landedCount = statusCounters[UAVInfo.LANDED]  
                    landingCount = statusCounters[UAVInfo.LANDING]  
                    states = nextStates  # 前进
                # 回合结束，汇总统计
                totalSuccess += landedCount
                totalCollision += collisionCount
                totalOver += overCount
                returns.append(episodeReturn)
                progressBar.set_postfix({
                    "episode": f"{episode}",
                    "return": f"{episodeReturn:.3f}",
                    "succ": f"{landedCount}",
                    "coll": f"{collisionCount}",
                    "over": f"{overCount}"
                })
                progressBar.update(1)
                if stopRequested["flag"]:
                    print("[测试] 终止于用户中断。")
                    break
    # 打印总览
    epRan = len(returns)
    meanRet = float(np.mean(returns)) if epRan > 0 else 0.0
    print("\n========== 测试汇总 ==========")
    print(f"回合数: {epRan}")
    print(f"平均回报: {meanRet:.3f}")
    print(f"成功总数: {totalSuccess}")
    print(f"碰撞总数: {totalCollision}")
    print(f"超步总数: {totalOver}")
    print("================================")


if __name__ == "__main__":
    test()

