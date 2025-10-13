#!/home/ldt/anaconda3/envs/deeplearning/bin/python
# -*- coding: utf-8 -*-
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
import hydra
import wandb
from tqdm import tqdm
import signal
import time
import numpy as np
from env.scripts.env import StaticObstacleEnv
from navigation.scripts.SAC import SAC
from uav.scripts.uav import UAVInfo
from tools import ReplayBuffer, moving_average, epsilon_annealing
from tools import save_checkPoint, load_checkPoint, cfg_get, to_plain_cfg, get_lr_safe


FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="train", version_base=None)
def main(cfg) -> None:
    """
    训练的主函数，Hydra注入config配置
    :param cfg: 配置
    :return: None
    """
    """读取wandb和断点相关配置，提供默认值"""
    wbEnabled = bool(cfg_get(cfg, "wandb.enabled", True))
    wbProject = cfg_get(cfg, "wandb.project", "XTDrone-DRL")
    wbEntity = cfg_get(cfg, "wandb.entity", "XTDrone-DRL-UAV-Navigation")
    wbName = cfg_get(cfg, "wandb.name", "navigation-training")
    wbMode = cfg_get(cfg, "wandb.mode", "online")
    wbDir = cfg_get(cfg, "wandb.dir", "../wandb")
    wbId = cfg_get(cfg, "wandb.id", None)
    wbResumeFlag = cfg_get(cfg, "wandb.resumeFlag", True)
    checkPointDir = cfg_get(cfg, "wandb.checkPointDir", "../checkPoints")
    checkPointPathConfig = cfg_get(cfg, "wandb.checkPointPath", "../checkPoints/kw9xf9k9.pt")
    saveEvery = int(cfg_get(cfg, "wandb.saveEvery", 10))
    """初始化wandb运行"""
    if wbEnabled:
        wandb.init(
            project=wbProject,
            entity=wbEntity,
            name=wbName,
            mode=wbMode,
            id=wbId if wbResumeFlag and wbId else None,  # 指定id以便合并到到同一个run
            resume="allow" if wbResumeFlag and wbId else None,  # 仅在提供id时允许自动续接
            dir=wbDir,
            config=to_plain_cfg(cfg),  # 将完整Hydra配置同步到run.config
        )
    """创建环境"""
    navigationAlgorithm = SAC(cfg)  # 实例化路径规划算法
    replayBuffer = ReplayBuffer(cfg.bufferSize)  # 实例化经验回放缓冲区
    env = StaticObstacleEnv(cfg)  # 实例化环境
    time.sleep(2)  # 必要，等待环境内部资源初始化缓冲完成
    """wandb与checkPoint的整合"""
    # 准备checkPoint的存储目录
    checkPointDir = os.path.abspath(os.path.join(os.getcwd(), checkPointDir))
    os.makedirs(checkPointDir, exist_ok=True)
    latestCheckPointPath = os.path.join(checkPointDir, "latest.pt")  # 最新的pt文件
    # 断点恢复
    startEpisodeIndex = 0  # 恢复的episode索引
    if wbResumeFlag:
        # 优先使用latest.pt
        loadFromPath = latestCheckPointPath if os.path.exists(latestCheckPointPath) else checkPointPathConfig
        if loadFromPath and os.path.exists(loadFromPath):
            try:
                startEpisodeIndex, replayBuffer = load_checkPoint(loadFromPath, navigationAlgorithm, replayBuffer)
                print(f"[wandb]加载断点：{loadFromPath}，从第{startEpisodeIndex}轮继续训练")
                if wbEnabled and wandb.run is not None:
                    wandb.summary["resumed_from"] = loadFromPath
            except Exception as e:
                print(f"[wandb]恢复失败，将从头开始：{e}")
    # 记录actor参数变化曲线
    watchRegistered = False  # 是否已注册watch
    totalEpisodes = int(cfg.numEpisodes)  # 总迭代轮数
    # 捕获 Ctrl+C（SIGINT），设置中断标记以触发保存断点并安全退出
    stopRequested = {"flag": False}

    def handle_sigint(signum, frame):
        stopRequested["flag"] = True
        print("\n[训练] 收到中断信号，将保存断点后退出...")

    signal.signal(signal.SIGINT, handle_sigint)
    """训练迭代"""
    try:
        with tqdm(total=totalEpisodes, desc="Training", initial=startEpisodeIndex) as progressBar:
            episodeIndex = startEpisodeIndex  # 迭代轮数
            while episodeIndex < totalEpisodes:
                """环境重置"""
                states =  env.reset()  # 获取状态信息（字典）
                time.sleep(2)  # 必要，等待环境内部资源初始化缓冲完成
                """初始化一些记录变量"""
                successCount = 0  # 成功到达目标点的无人机个数
                episodeReturn = 0  # 迭代的奖励总和
                collisionCount = 0  # 发生碰撞的无人机个数
                overCount = 0  # 超出最大步长的无人机个数
                doneCount = 0  # 完成的无人机个数（包括达到目标点、发生碰撞、超出最大步长）
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
                        if state is None or nextStates[i] is None:
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
                    # 首次前向完成后，再注册 watch（仅一次）
                    if wbEnabled and wandb.run is not None and not watchRegistered:
                        try:
                            modulesToWatch = []
                            actorModule = getattr(navigationAlgorithm, "actor", None)
                            if actorModule is not None:
                                modulesToWatch.append(actorModule)
                            critic1 = getattr(navigationAlgorithm, "critic1", None)
                            critic2 = getattr(navigationAlgorithm, "critic2", None)
                            if critic1 is not None: modulesToWatch.append(critic1)
                            if critic2 is not None: modulesToWatch.append(critic2)
                            for m in modulesToWatch:
                                wandb.watch(m, log="all", log_freq=5)
                            watchRegistered = True
                        except Exception as e:
                            print(f"[wandb]watch延后注册失败：{e}")
                """wandb指标记录"""
                if wbEnabled and wandb.run is not None:
                    # 记录指标
                    logData = {
                        "episode": episodeIndex,
                        "episodeReturn": float(episodeReturn),
                        "successCount": int(successCount),
                        "collisionCount": int(collisionCount),
                        "overCount": int(overCount),
                        "lr/actor": get_lr_safe(getattr(navigationAlgorithm, "actorScheduler", None)),
                        "lr/critic1": get_lr_safe(getattr(navigationAlgorithm, "criticScheduler1", None)),
                        "lr/critic2": get_lr_safe(getattr(navigationAlgorithm, "criticScheduler2", None)),
                    }
                    wandb.log(logData, step=episodeIndex)  #  wandb记录
                """周期性保存checkPoint，并可上传为artifact"""
                if (episodeIndex + 1) % max(1, saveEvery) == 0 or episodeIndex == totalEpisodes - 1:
                    # 保存checkPoint
                    save_checkPoint(
                        latestCheckPointPath,
                        episodeIndex,
                        navigationAlgorithm,
                        replayBuffer,
                        extraInfo={"wandb_run_id": wandb.run.id if wbEnabled and wandb.run else None}
                    )
                    # 保存快照
                    if episodeIndex + 1 % 100 == 0:
                        snapshotPath = os.path.join(checkPointDir, f"ep_{episodeIndex}.pt")
                        save_checkPoint(snapshotPath, episodeIndex, navigationAlgorithm, replayBuffer)
                    # 上传到wandb
                    if wbEnabled and wandb.run is not None:
                        try:
                            artifact = wandb.Artifact("sac-checkpoints", type="model")
                            artifact.add_file(latestCheckPointPath, name="latest.pt")
                            artifact.add_file(snapshotPath, name=f"ep_{episodeIndex}.pt")
                            wandb.log_artifact(artifact)
                        except Exception:
                            pass
                """打印每一步的训练信息"""
                progressBar.set_postfix({
                    "episode": f"{episodeIndex}",
                    "return": f"{episodeReturn:.3f}",
                    "successCount": f"{successCount}",
                    "collisionCount": f"{collisionCount}",
                    "overCount": f"{overCount}"
                })
                progressBar.update(1)
                episodeIndex += 1
                """收到中断的处理"""
                if stopRequested["flag"]:
                    save_checkPoint(
                        latestCheckPointPath,
                        episodeIndex,
                        navigationAlgorithm,
                        replayBuffer,
                        extraInfo={"wandb_run_id": wandb.run.id if wbEnabled and wandb.run else None}
                    )
                    print(f"[训练] 已保存断点到 {latestCheckPointPath}")
                    break
    finally:
        # 关闭 wandb 运行并写入汇总信息（标记最后完成的 episode）
        if wbEnabled and wandb.run is not None:
            wandb.summary["final_episode"] = startEpisodeIndex if stopRequested["flag"] else totalEpisodes
            wandb.finish()


if __name__ == "__main__":
    main()