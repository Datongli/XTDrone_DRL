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
from uav.scripts.uav import UAVInfo
from hydra.utils import to_absolute_path
from train.scripts.tools import cfg_get, to_plain_cfg, load_checkPoint
from train.scripts.tools import set_gazebo_unlimit, kill_gzclient


def load_pth_models(checkpointDirctory: str, algorithm: DDPG | SAC, device):
    """
    加载 .pth 格式的单独模型文件（DDPG 专用）
    :param checkpointDirctory: checkpoint 目录路径
    :param algorithm: 算法实例（DDPG 或 SAC）
    :param device: 设备
    :return: 是否加载成功
    """
    try:
        # 查找 .pth 文件
        actorPath = os.path.join(checkpointDirctory, "actor.pth")
        targetActorPath = os.path.join(checkpointDirctory, "target_actor.pth")
        criticPath = os.path.join(checkpointDirctory, "critic.pth")
        targetCriticPath = os.path.join(checkpointDirctory, "target_critic.pth")
        pthLoad = {
            "actor": actorPath,
            "targetActor": targetActorPath,
            "critic": criticPath,
            "targetCritic": targetCriticPath
        }
        # 检查文件是否存在
        requiredFiles = [actorPath, targetActorPath, criticPath, targetCriticPath]
        missingFiles = [f for f in requiredFiles if not os.path.exists(f)]
        if missingFiles:
            print(f"[测试] 缺少模型文件: {missingFiles}")
            return False
        # 加载模型参数
        print(f"[测试] 正在加载 DDPG 模型文件...")
        for name, pth in pthLoad.items():
            if os.path.exists(pth):
                chechPoint = torch.load(pthLoad[name], map_location=device)
                algorithm.netDict[name].load_state_dict(chechPoint["model"])
                print(f"[测试] ✓ 已加载 {name}: {pth}")
        # if hasattr(algorithm, 'actor'):
        #     algorithm.actor.load_state_dict(torch.load(actorPath, map_location=device))
        #     print(f"[测试] ✓ 已加载 actor: {actorPath}")
        # if hasattr(algorithm, 'targetActor'):
        #     algorithm.targetActor.load_state_dict(torch.load(targetActorPath, map_location=device))
        #     print(f"[测试] ✓ 已加载 targetActor: {targetActorPath}")
        # if hasattr(algorithm, 'critic'):
        #     algorithm.critic.load_state_dict(torch.load(criticPath, map_location=device))
        #     print(f"[测试] ✓ 已加载 critic: {criticPath}")
        # if hasattr(algorithm, 'targetCritic'):
        #     algorithm.targetCritic.load_state_dict(torch.load(targetCriticPath, map_location=device))
        #     print(f"[测试] ✓ 已加载 targetCritic: {targetCriticPath}")
        return True
    except Exception as e:
        print(f"[测试] 加载 .pth 模型失败: {e}")
        import traceback
        traceback.print_exc()
        return False


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
    """创建环境"""
    # 根据配置选择算法
    algorithmType = cfg_get(cfg, "eval.navigationModel", "SAC")
    if algorithmType == "DDPG":
        navigationAlgorithm = DDPG(cfg)
    else:
        navigationAlgorithm = SAC(cfg)
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
        # actor模型
        actorModule = getattr(navigationAlgorithm, "actor", None)
        if actorModule is not None and hasattr(actorModule, "eval"):  actorModule.eval()
        # targetActor模型
        targetActorModule = getattr(navigationAlgorithm, "targetActor", None)
        if targetActorModule is not None and hasattr(targetActorModule, "eval"):  targetActorModule.eval()
        # critic模型
        critic = getattr(navigationAlgorithm, "critic", None)
        critic1 = getattr(navigationAlgorithm, "critic1", None)
        critic2 = getattr(navigationAlgorithm, "critic2", None)
        if critic is not None and hasattr(critic, "eval"):  critic.eval()
        if critic1 is not None and hasattr(critic1, "eval"):  critic1.eval()
        if critic2 is not None and hasattr(critic2, "eval"):  critic2.eval()
        # targetCritic模型
        targetCritic = getattr(navigationAlgorithm, "targetCritic", None)
        if targetCritic is not None and hasattr(targetCritic, "eval"):  targetCritic.eval()
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
                while doneCount < cfg.uav.uavNums:
                    doneCount = 0
                    # 获得状态
                    states = [s for s in states if s is not None]
                    # 选择动作
                    actions = navigationAlgorithm.take_action(states)
                    actions = np.array(actions, dtype=np.float32)
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
                    }
                    for uav in env.uavs:
                        statusCounters[uav.info] += 1
                        if uav.done:
                            doneCount += 1
                    successCount = statusCounters[UAVInfo.SUCCESS]
                    collisionCount = statusCounters[UAVInfo.COLLISION]
                    overCount = statusCounters[UAVInfo.STEP_OVER]
                    states = nextStates  # 前进
                # 回合结束，汇总统计
                totalSuccess += successCount
                totalCollision += collisionCount
                totalOver += overCount
                returns.append(episodeReturn)
                progressBar.set_postfix({
                    "episode": f"{episode}",
                    "return": f"{episodeReturn:.3f}",
                    "succ": f"{successCount}",
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

