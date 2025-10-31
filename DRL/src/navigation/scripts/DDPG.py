import random
import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
import torch.nn.init as init


class ActorTracing(torch.nn.Module):
    """用于寻迹部分的actor网络"""
    def __init__(self, numIn: int, numOut: int, hiddenDim: int=32, 
                 activation: callable=F.relu, outFunction: callable=lambda x: x) -> None:
        """
        初始化actor网络
        :param numIn: 输入维度
        :param numOut: 输出维度
        :param hiddenDim: 隐藏层维度
        :param activation: 激活函数
        :param outFunction: 输出函数
        """
        super(ActorTracing, self).__init__()
        self.fc1 = torch.nn.Linear(numIn, hiddenDim)
        self.fc2 = torch.nn.Linear(hiddenDim, hiddenDim)
        self.fc3 = torch.nn.Linear(hiddenDim, hiddenDim)
        self.fc4 = torch.nn.Linear(hiddenDim, hiddenDim)
        self.fc5 = torch.nn.Linear(hiddenDim, numOut)
        self.dropout = torch.nn.Dropout(p=0.1)
        self.activation = activation
        self.outFunction = outFunction

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        前向传播函数
        :param x: 输入张量
        :return: 输出张量
        """
        x = self.fc1(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc2(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc3(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc4(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc5(x)
        x = self.outFunction(x)
        return x


class ActorAvoid(torch.nn.Module):
    """用于避障部分的actor网络"""
    def __init__(self, numIn: int, numOut: int, hiddenDim: int=64, 
                 activation: callable=F.relu, outFunction: callable=lambda x: x) -> None:
        """
        初始化actor网络
        :param numIn: 输入维度
        :param numOut: 输出维度
        :param hiddenDim: 隐藏层维度
        :param activation: 激活函数
        :param outFunction: 输出函数
        """
        super(ActorAvoid, self).__init__()
        self.conv1 = torch.nn.Conv1d(1, 16, 3, stride=1, padding=1)
        self.bn1 = nn.BatchNorm1d(16)
        self.conv2 = torch.nn.Conv1d(16, 32, 3, stride=1, padding=1)
        self.bn2 = nn.BatchNorm1d(32)
        self.conv3 = torch.nn.Conv1d(32, 64, 3, stride=1, padding=1)
        self.bn3 = nn.BatchNorm1d(64)
        self.conv4 = torch.nn.Conv1d(64, 32, 3, stride=1, padding=1)
        self.bn4 = nn.BatchNorm1d(32)
        self.conv5 = torch.nn.Conv1d(32, 16, 3, stride=1, padding=1)
        self.bn5 = nn.BatchNorm1d(16)
        self.conv6 = torch.nn.Conv1d(16, 1, 3, stride=1, padding=1)
        self.fc6 = torch.nn.Linear(10, numOut)
        self.dropout = torch.nn.Dropout(p=0.1)
        self.bn = nn.BatchNorm1d(hiddenDim)
        self.activation = activation
        self.outFunction = outFunction

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        前向传播函数
        :param x: 输入张量
        :return: 输出张量
        """
        x = self.conv1(x)
        x = self.activation(x)
        x = self.conv2(x)
        x = self.activation(x)
        x = self.conv3(x)
        x = self.activation(x)
        x = self.conv4(x)
        x = self.activation(x)
        x = self.conv5(x)
        x = self.activation(x)
        x = self.conv6(x)
        x = self.fc6(x)
        x = self.outFunction(x)
        return x


class Actor(torch.nn.Module):
    """合并寻迹和避障两个网络的actor动作网络"""
    def __init__(self, numIn: int, numOut: int, hiddenDim: int=128, 
                 activation: callable=F.relu, outFunction: callable=lambda x: x) -> None:
        """
        初始化这个两层神经网络
        :param numIn: 输入的节点数量
        :param numOut: 输出的节点数量
        :param hiddenDim: 隐藏层节点数量
        :param activation: 激活函数
        :param outFunction: 输出的函数。这里使用了一个匿名函数 (lambda 函数)，该函数接受一个参数 x，并直接返回 x。
        例如，如果你创建了一个 Actor 类的实例时提供了一个不同的输出函数，比如 outFunction=lambda x: torch.sigmoid(x)，
        那么在前向传播过程中就会应用该函数来处理神经网络的输出。这使得 LayerFC 类更加灵活，能够适应不同的需求。
        :param target: 是否为目标网络
        """
        super(Actor, self).__init__()
        # 一些全连接层的定义
        self.fc1 = torch.nn.Linear(numOut * 2, hiddenDim)
        self.fc2 = torch.nn.Linear(hiddenDim, hiddenDim)
        self.fc3 = torch.nn.Linear(hiddenDim, hiddenDim)
        self.fc4 = torch.nn.Linear(hiddenDim, hiddenDim)
        self.fc5 = torch.nn.Linear(hiddenDim, numOut)
        # 寻迹网络
        self.actor_tracing = ActorTracing(3, numOut, 32, activation=activation, outFunction=outFunction)
        # 避障网络
        self.actor_avoid = ActorAvoid(6, numOut, 32, activation=activation, outFunction=outFunction)
        # 其他部分
        self.dropout = torch.nn.Dropout(p=0.1)
        self.activation = activation
        self.outFunction = outFunction

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        前向传播函数
        :param x: 输入张量
        :return: 输出张量
        """
        x1 = x[:, :, :3]
        x2 = x[:, :, 3:]
        # """计算只寻迹的动作"""
        x1 = self.actor_tracing(x1)
        # """计算只避障的动作"""
        x2 = self.actor_avoid(x2)
        # """合并"""
        x_all = torch.cat((x1, x2), dim=2)
        x = self.fc1(x_all)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc2(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc3(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc4(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc5(x)
        x = self.outFunction(x)
        return x


class Critic(torch.nn.Module):
    def __init__(self, numIn: int, numOut: int, hiddenDim: int=128, 
                 activation: callable=F.relu, outFunction: callable=lambda x: x) -> None:
        """
        初始化这个两层神经网络
        :param numIn: 输入的节点数量
        :param numOut: 输出的节点数量
        :param hiddenDim: 隐藏层节点数量
        :param activation: 激活函数
        :param outFunction: 输出的函数。这里使用了一个匿名函数 (lambda 函数)，该函数接受一个参数 x，并直接返回 x。
        例如，如果你创建了一个 Critic 类的实例时提供了一个不同的输出函数，比如 outFunction=lambda x: torch.sigmoid(x)，
        那么在前向传播过程中就会应用该函数来处理神经网络的输出。这使得 LayerFC 类更加灵活，能够适应不同的需求。
        """
        super(Critic, self).__init__()
        self.fc1 = torch.nn.Linear(numIn, hiddenDim)
        self.fc2 = torch.nn.Linear(hiddenDim, hiddenDim)
        self.fc3 = torch.nn.Linear(hiddenDim, hiddenDim)
        self.fc4 = torch.nn.Linear(hiddenDim, hiddenDim)
        self.fc5 = torch.nn.Linear(hiddenDim, numOut)
        self.numOut = numOut
        self.activation = activation
        self.outFunction = outFunction
        # 定义可以训练的均值和方差，用于在输入时归一化输入，有利于训练的稳定
        self.mean = nn.Parameter(torch.zeros((numIn,)), requires_grad=True)
        self.std = nn.Parameter(torch.ones((numIn,)), requires_grad=True)
        self.dropout = torch.nn.Dropout(p=0.1)

    def init_weights(self) -> None:
        # 使用 Xavier/Glorot 初始化
        for layer in [self.fc1, self.fc2, self.fc3, self.fc4, self.fc5]:
            if isinstance(layer, nn.Linear):
                init.xavier_uniform_(layer.weight)
                init.normal_(layer.bias, mean=0, std=1)

    def input_norm(self, x: torch.Tensor) -> torch.Tensor:
        """
        用于归一化输入，使用的mean和std均是可以训练的参数
        :param x:输入的数据
        :return:归一化之后的输入
        """
        return (x - self.mean) / self.std

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        前向传播函数
        :param x: 输入张量
        :return: 输出张量
        """
        x = self.fc1(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc2(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc3(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc4(x)
        x = self.activation(x)
        x = self.dropout(x)
        x = self.fc5(x)
        x = self.outFunction(x)
        return x


class DDPG:
    """
    DDPG算法
    """
    def __init__(self, cfg, numInActor: int=21, numOutActor: int=3, numInCritic: int=24, hiddenDim: int=64, discrete: bool=False,
                 actionBound: float=2.0, sigma: float=0.01, actorLearningRate: float=0.001, criticLearningRate: float=0.001, 
                 tau: float=0.05, gamma: float=0.99, maxEpsEpisode: int=0, minEps: float=0,
                 wd: float=0.0) -> None:
        """
        初始化DDPG算法
        :param cfg:配置文件
        :param numInActor:策略网络输入节点（状态维度）
        :param numOutActor:策略网络输出节点（动作维度）
        :param numInCritic:价值网络输入节点
        :param hiddenDim:隐藏层节点
        :param discrete:是否为离散
        :param actionBound:环境可以接受的动作最大值
        :param sigma:高斯噪声的标准差
        :param actorLearningRate:策略网络学习率
        :param criticLearningRate:价值网络学习率
        :param tau:更新目标网络的参数值
        :param gamma:折扣因子
        :param maxEpsEpisode:最大贪心次数
        :param minEps:最小贪心概率
        :param wd:正则化强度
        """
        self.cfg = cfg
        self.device = self.cfg.device
        outFunction = (lambda x: x) if discrete else (lambda x: torch.tanh(x) * actionBound)  # 输出函数
        # DDPG算法的四个网络
        self.actor = Actor(numInActor, numOutActor, hiddenDim, activation=nn.PReLU(), outFunction=outFunction).to(self.device)
        self.targetActor = Actor(numInActor, numOutActor, hiddenDim, activation=nn.PReLU(), outFunction=outFunction).to(self.device)
        self.critic = Critic(numInCritic, 1, hiddenDim, activation=nn.PReLU(), outFunction=lambda x: x).to(self.device)
        self.targetCritic = Critic(numInCritic, 1, hiddenDim, activation=nn.PReLU(), outFunction=lambda x: x).to(self.device)
        # 初始化目标价值网络并设置和价值网络相同的参数
        self.targetCritic.load_state_dict(self.critic.state_dict())
        # 初始化目标策略网络并设置和策略相同的参数
        self.targetActor.load_state_dict(self.actor.state_dict())
        # 优化器
        self.actorOptimizer = torch.optim.Adam(self.actor.parameters(), lr=actorLearningRate, weight_decay=wd)
        self.criticOptimizer = torch.optim.Adam(self.critic.parameters(), lr=criticLearningRate, weight_decay=wd)
        # 学习率衰减
        self.actorScheduler = torch.optim.lr_scheduler.StepLR(self.actorOptimizer, step_size=500, gamma=0.1)
        self.criticScheduler = torch.optim.lr_scheduler.StepLR(self.criticOptimizer, step_size=500, gamma=0.1)
        self.gamma = gamma
        # 高斯噪声的标准差，均值直接设为0
        self.sigma = sigma
        self.actionBound = actionBound
        self.tau = tau
        self.actionDim = numOutActor
        self.maxEpsEpisode = maxEpsEpisode
        self.minEps = minEps    
        self.step = 0
        self.train = True
        self.action_flag = True
        # 承接预训练的模型参数
        self.netDict = {
            'actor': self.actor,
            'targetActor': self.targetActor,
            'critic': self.critic,
            'targetCritic': self.targetCritic
        }
        # actor网络字典，用于承接预训练好的寻迹和避障的参数
        self.actorDict = {
            'actorTracing': self.actor.actor_tracing,
            'actorAvoid': self.actor.actor_avoid,
            'targetActorTracing': self.targetActor.actor_tracing,
            'targetActorAvoid': self.targetActor.actor_avoid
        }
        # 优化器字典，用于承接优化器
        self.netOptim = {
            'actor': self.actorOptimizer,
            'critic': self.criticOptimizer
        }
        
    def take_action(self, state: list[dict]) -> np.ndarray:
        """
        选择动作
        :param state:当前状态
        :return:动作
        """
        # 特征提取
        state = self._feature_extraction(state)
        # 选择动作
        action = self._normal_take_action(state)
        return action
    
    def _feature_extraction(self, uavInformations: list[dict]) -> torch.Tensor:
        """
        特征提取函数
        :param uavInformations: UAV信息列表
        :return: 提取特征后的状态张量
        """
        """过滤无效条目"""
        pairedList = []
        for informationItem in uavInformations:
            if not informationItem or not isinstance(informationItem, dict):
                continue
            sensorDataRaw = informationItem.get("sensorData", None)  
            uavStateRaw = informationItem.get("uavState", None)
            uavZ = informationItem.get("uavZ", None)
            if sensorDataRaw is None or uavStateRaw is None or uavZ is None:
                continue
            pairedList.append((sensorDataRaw, uavStateRaw, uavZ))
        if not pairedList:
            raise ValueError("_feature_extraction: empty inputs")
        """提取传感器数据和状态数据"""
        stateList = []  # 存储纯 Python list
        for sensorDataRaw, uavStateRaw, uavZ in pairedList:
            # 确保传感器数据是 numpy 数组
            sensorArray = np.asarray(sensorDataRaw, dtype=np.float32)
            numPoints = len(sensorArray)
            # 提取 8 个方向的距离
            angles = [-135, -90, -45, -180, 0, 135, 90, 45]
            indices = [self._angle_to_index(ang, numPoints) for ang in angles]
            # 提取距离值并转换为 Python float
            distances = [float(sensorArray[idx]) for idx in indices]
            # 对四个角的数据进行阈值处理（左后、左前、右后、右前）
            for i in [0, 2, 5, 7]:
                if distances[i] < 1.0:
                    distances[i] = 0.0
                else:
                    distances[i] = 1000.0
            # 插入高度信息
            sensorFeatures = distances.copy()
            sensorFeatures.insert(4, float(uavZ))  # 下（当前高度）
            sensorFeatures.insert(5, float(self.cfg.env.height - uavZ))  # 上（到天花板距离）
            # 提取 UAV 状态（前3个元素：通常是 x, y, z 或 x, y, yaw）
            if isinstance(uavStateRaw, (list, tuple)):
                uav_state = [float(x) for x in uavStateRaw[:3]]
            elif isinstance(uavStateRaw, np.ndarray):
                uav_state = [float(x) for x in uavStateRaw.flat[:3]]
            else:
                # 如果是单个数值
                uav_state = [float(uavStateRaw), 0.0, 0.0]
            # 拼接状态：UAV状态(3) + 传感器特征(10) = 13维
            combined_state = uav_state + sensorFeatures
            stateList.append(combined_state)
        # 直接从纯 Python list 创建 tensor（避免 NumPy 类型推断问题）
        states = torch.tensor(stateList, dtype=torch.float32, device=self.device)
        # 增加时间步维度 (batch_size, 1, feature_dim)
        states = states.unsqueeze(1)
        return states
    
    def _normal_take_action(self, state: torch.Tensor) -> np.ndarray:
        """
        正常情况下的动作选择函数
        :param state:当前状态
        :return:动作
        """
        # 获取动作张量
        actionTensor = self.actor(state).detach()
        # 要添加的 yaw 值
        yaw = np.pi / 2
        # 在 PyTorch 中添加 yaw 维度
        yawTensor = torch.full(
            (actionTensor.shape[0], actionTensor.shape[1], 1),
            yaw,
            dtype=torch.float32,
            device=actionTensor.device
        )
        # 在 PyTorch 中拼接
        actionWthYaw = torch.cat([actionTensor, yawTensor], dim=-1)
        # 去除时间步维度 (batch_size, 1, 4) -> (batch_size, 4)
        actionWithYaw = actionWthYaw.squeeze(1)
        # 转换为 NumPy 数组
        action = actionWithYaw.cpu().numpy().astype(np.float32)
        return action
    
    @staticmethod
    def _angle_to_index(angleDeg: float| int, numPoints: int) -> int:
        """
        将角度（度）转换为数组索引
        :param angleDeg: 角度（度）
        :param numPoints: 数组长度
        :return: 数组索引
        """
        angle_rad = np.deg2rad(angleDeg)
        index = int((angle_rad + np.pi) / (2 * np.pi) * numPoints)
        # 确保索引在有效范围内
        return np.clip(index, 0, numPoints - 1)



if __name__ == "__main__":
    pass