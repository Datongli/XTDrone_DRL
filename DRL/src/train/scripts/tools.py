"""
工具函数文件
"""
from tqdm import tqdm
import numpy as np
import torch
import collections
import random


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