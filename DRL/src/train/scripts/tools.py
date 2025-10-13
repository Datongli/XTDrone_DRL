"""
工具函数文件
"""
from tqdm import tqdm
import numpy as np
import torch
import collections
import random
import os
import pickle
import time
import pickle


class ReplayBuffer:
    """
    经验回放池
    """

    def __init__(self, capacity: int) -> None:
        # 一个先进先出的队列
        # 创建了一个具有固定容量（capacity）的双端队列（deque）
        # maxlen 参数指定了经验缓冲区的最大容量，即队列中元素的最大数量。
        # 当缓冲区达到最大容量时，添加新元素会导致旧元素从队列的另一端被移除，以保持缓冲区不超过设定的最大值。
        self.buffer = collections.deque(maxlen=capacity)

    def add(self, state: dict, action: np.array, reward: float, nextState: dict, done: bool) -> tuple:
        """
        将数据加入buffer中
        :param state: 状态
        :param action: 动作
        :param reword: 奖励
        :param next_state:动作之后的下一状态
        :param done:环境是否结束
        :return: 更新容器
        """
        self.buffer.append((state.copy(), action.copy(), reward, nextState.copy(), done))

    def sample(self, batchSize: int) -> tuple:
        """
        从缓冲区挑选数量为batchSize的样本
        :param batchSize: 要挑选的数量
        :return: 状态，动作，奖励，下一状态，done
        """
        # 从经验回放缓冲区中无放回地随机选择batch_size个样本（但是选择后缓冲区中经验数量不变）
        transitions = random.sample(self.buffer, batchSize)
        # 将 transitions 中的元素按列解压缩成五个列表
        # 并分别赋值给 state、action、reward、next_state 和 done。
        state, action, reward, nextState, done = zip(*transitions)
        return np.array(state), action, reward, np.array(nextState), done

    def size(self) -> int:
        """
        读取目前缓冲区中数据的数量
        :return: 缓冲区中数据数量
        """
        return len(self.buffer)
    

def moving_average(a: np.array, windowSize: int) -> np.array:
    """
    绘图平滑窗口函数
    :param a: 输入数组
    :param window_size: 窗口大小
    :return: 平滑后的数组
    """
    cumulative_sum = np.cumsum(np.insert(a, 0, 0))
    middle = (cumulative_sum[windowSize:] - cumulative_sum[:-windowSize]) / windowSize
    r = np.arange(1, windowSize - 1, 2)
    begin = np.cumsum(a[:windowSize - 1])[::2] / r
    end = (np.cumsum(a[:-windowSize:-1])[::2] / r)[::-1]
    return np.concatenate((begin, middle, end))


def epsilon_annealing(iEpsiode: int, maxEpisode: int, minEps: float) -> float:
    """
    伊布西龙衰减函数，用于计算贪心概率
    :param i_epsiode:当前的迭代次数
    :param max_episode:最大的贪心次数
    :param min_eps:最小的伊布西龙值，表示希望在训练结束时，模型趋向于完全贪心
    :return:贪心概率
    """
    # 计算斜率，用于线性衰减
    slope = (minEps - 1.0) / maxEpisode
    # 其实在当前的斜率（-0.099）下，迭代10次后，就已经为min_eps了
    # 是一个先线性下降，再恒定的过程，最后恒定在0.01
    retEps = max(slope * iEpsiode + 1.0, minEps)
    return retEps


def cfg_get(cfg, dottedKey: str, default: any)-> any:
    """
    安全读取配置文件中的参数
    支持用点分隔的多级键
    若任一层不存在或者访问失败，返回default，避免KeyError/AttributeError
    :param cfg: 配置文件对象
    :param dottedKey: 键路径
    :param default: 默认值
    :return: 参数值
    """
    try:
        cursor = cfg  # 游标
        # 遍历键路径中的每个键
        for key in dottedKey.split('.'):
            cursor = getattr(cursor, key)
        return cursor
    except Exception:
        # 若任意层不存在或者访问失败，返回默认值
        return default

def to_plain_cfg(cfg) -> dict[str, any]:
    """
    将 OmegaConf 配置对象转换为可序列化的 Python dict（resolve 插值）
    在无法导入omegaconf或转换异常时返回空字典，避免wandb.init崩溃
    :param cfg: 配置文件对象
    :return: 可序列化的 Python dict
    """
    try:
        from omegaconf import OmegaConf
        return OmegaConf.to_container(cfg, resolve=True)
    except Exception:
        return {}
    

def safe_state_dict(targetObject: torch.nn.Module | None) -> dict | None:
    """
    安全获取模型的状态字典（state_dict）
    若模型未定义状态字典或获取过程中发生异常，返回None，避免崩溃
    :param targetObject: 模型对象
    :return: 模型的状态字典或None
    """
    try:
        return targetObject.state_dict()
    except Exception:
        return None
    

def safe_load_state(targetObject: torch.nn.Module | None, stateDict: dict | None) -> None:
    """
    安全加载模型的状态字典（state_dict）
    若模型未定义加载状态字典方法或加载过程中发生异常，忽略，避免崩溃
    :param targetObject: 模型对象
    :param stateDict: 状态字典
    :return: None
    """
    if targetObject is not None and hasattr(targetObject, "load_state_dict") and stateDict is not None:
        try:
            targetObject.load_state_dict(stateDict)
        except Exception:
            pass


def get_lr_safe(lrScheduler: torch.optim.lr_scheduler._LRScheduler | None) -> float | None:
    """
    安全读取学习率调度器当前学习率（兼容无调度器或实现差异）。
    优先取 get_last_lr()[0]，失败时返回 None。
    :param lrScheduler: 学习率调度器对象
    :return: 当前学习率或None
    """
    try:
        return float(lrScheduler.get_last_lr()[0])
    except Exception:
        return None
    

def save_checkPoint(checkPointPath: str, episodeIndex: int, navigationAlgorithm, replayBuffer: ReplayBuffer, extraInfo: dict[str, any] = None):
    """
    保存模型检查点
    :param checkpointPath: 检查点保存路径
    :param episodeIndex: 当前迭代次数
    :param navigationAlgorithm: 导航算法对象(网络、优化器、alpha参数等)
    :param replayBuffer: 经验回放缓冲区对象
    :param extraInfo: 额外的信息字典
    :return: None
    """
    os.makedirs(os.path.dirname(checkPointPath), exist_ok=True)
    # 尝试序列化经验回放
    try:
        replayBufferBytes = pickle.dumps(replayBuffer)
    except Exception:
        replayBufferBytes = None
    # 检查点状态
    checkPointState = {
        "episode": int(episodeIndex),
        "timeStamp": time.time(),
        "navigartionAlgorithm": {
            # 模型权重
            "actor": safe_state_dict(getattr(navigationAlgorithm, "actor", None)),
            "critic1": safe_state_dict(getattr(navigationAlgorithm, "critic1", None)),
            "critic2": safe_state_dict(getattr(navigationAlgorithm, "critic2", None)),
            # 优化器
            "actorOptimizer": safe_state_dict(getattr(navigationAlgorithm, "actorOptimizer", None)),
            "criticOptimizer1": safe_state_dict(getattr(navigationAlgorithm, "criticOptimize1", None)),
            "criticOptimizer2": safe_state_dict(getattr(navigationAlgorithm, "criticOptimizer2", None)),
            # 学习率
            "actorScheduler": safe_state_dict(getattr(navigationAlgorithm, "actorScheduler", None)),
            "criticScheduler1": safe_state_dict(getattr(navigationAlgorithm, "criticScheduler1", None)),
            "criticScheduler2": safe_state_dict(getattr(navigationAlgorithm, "criticScheduler2", None)),
            # alpha参数
            "logAlpha": float(getattr(navigationAlgorithm, "logAlpha", None)),
            "logAlphaOptimizer": safe_state_dict(getattr(navigationAlgorithm, "logAlphaOptimizer", None)),
        },
        "replayBuffer": replayBufferBytes
    }
    if extraInfo:
        checkPointState["extraInfo"] = extraInfo
    # --- 修复 4GiB 序列化限制：强制使用更高的 pickle 协议 ---
    protocol = 5 if getattr(pickle, "HIGHEST_PROTOCOL", 4) >= 5 else 4
    try:
        # 新版 PyTorch 支持 pickle_protocol 参数
        torch.save(checkPointState, checkPointPath, pickle_protocol=protocol)
    except TypeError:
        # 兼容旧版 PyTorch：回退到直接用 pickle.dump
        with open(checkPointPath, "wb") as f:
            pickle.dump(checkPointState, f, protocol=protocol)


def load_checkPoint(checkPointPath: str, navigationAlgorithm: any, replayBuffer: ReplayBuffer) -> tuple:
    """
    从checkPoint文件恢复训练断点
    :param checkPointPath: 检查点文件路径
    :param navigationAlgorithm: 导航算法对象(网络、优化器、alpha参数等)
    :param replayBuffer: 经验回放缓冲区对象，若replay buffer可反序列化则直接替换原对象
    :return: (episodeIndex, replayBufferObject)
    """
    """加载检查点"""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 判断是否有可用的设备
    checkPointState = torch.load(checkPointPath, map_location=device, weights_only=False)  # 加载检查点状态
    navigationState = checkPointState.get("navigartionAlgorithm", {})  # 获取导航算法状态
    """加载导航算法状态"""
    safe_load_state(getattr(navigationAlgorithm, "actor", None), navigationState.get("actor", None))
    safe_load_state(getattr(navigationAlgorithm, "critic1", None), navigationState.get("critic1", None))
    safe_load_state(getattr(navigationAlgorithm, "critic2", None), navigationState.get("critic2", None))
    """加载导航算法优化器状态"""
    safe_load_state(getattr(navigationAlgorithm, "actorOptimizer", None), navigationState.get("actorOptimizer", None))
    safe_load_state(getattr(navigationAlgorithm, "criticOptimizer1", None), navigationState.get("criticOptimizer1", None))
    safe_load_state(getattr(navigationAlgorithm, "criticOptimizer2", None), navigationState.get("criticOptimizer2", None))
    """加载导航算法学习率调度器状态"""
    safe_load_state(getattr(navigationAlgorithm, "actorScheduler", None), navigationState.get("actorScheduler", None))
    safe_load_state(getattr(navigationAlgorithm, "criticScheduler1", None), navigationState.get("criticScheduler1", None))
    safe_load_state(getattr(navigationAlgorithm, "criticScheduler2", None), navigationState.get("criticScheduler2", None))
    """加载导航算法alpha参数状态"""
    safe_load_state(getattr(navigationAlgorithm, "logAlpha", None), navigationState.get("logAlpha", None))
    safe_load_state(getattr(navigationAlgorithm, "logAlphaOptimizer", None), navigationState.get("logAlphaOptimizer", None))
    """恢复replayBuffer状态"""
    # 若反向序列化失败，沿用传入的replayBuffer对象
    replayBufferBytes = checkPointState.get("replayBuffer", None)
    if replayBufferBytes is not None:
        try:
            # 尝试反序列化replay buffer
            loadedReplayBuffer = pickle.loads(replayBufferBytes)
            return checkPointState.get("episode", 0), loadedReplayBuffer
        except Exception:
            pass
    return checkPointState.get("episode", 0), replayBuffer