"""
SAC算法主要代码
"""
import torch
import torch.nn.functional as F
import torch.nn as nn
from torch.distributions import Normal
import numpy as np


class DepthCameraFeatureExtractor(nn.Module):
    """
    深度相机特征提取器
    现在主要使用卷积网络提取深度相机特征
    """
    def __init__(self, cfg) -> None:
        """
        构造函数
        :param cfg: 配置
        :return: None
        """
        super().__init__()
        """可能需要的参数"""
        self.cfg = cfg
        """特征预处理部分"""
        # 有可能需要在这里添加一些预处理
        """网络层"""
        # 卷积层
        self.conv1 = nn.LazyConv2d(out_channels=4, kernel_size=[4, 4], stride=[2, 2], padding=[1, 1])
        self.conv2 = nn.LazyConv2d(out_channels=16, kernel_size=[4, 4], stride=[2, 2], padding=[1, 1])
        self.conv3 = nn.LazyConv2d(out_channels=32, kernel_size=[4, 4], stride=[2, 2], padding=[1, 1])
        # 展平
        self.flatten = nn.Flatten()
        # 归一化层
        self.bn1 = nn.LazyBatchNorm2d()
        self.bn2 = nn.LazyBatchNorm2d()
        self.bn3 = nn.LazyBatchNorm2d()
        # 池化
        self.pool = nn.MaxPool2d(kernel_size=[2, 2], stride=[2, 2])
        # 激活层
        self.relu = nn.ReLU()
        # 全连接层
        self.fc1 = nn.LazyLinear(out_features=256)
        self.fc2 = nn.LazyLinear(out_features=128)
        self.fc3 = nn.LazyLinear(out_features=64)
        self.fc4 = nn.LazyLinear(out_features=4)
        # dropout层
        self.dropout = nn.Dropout(p=0.1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """前向传播
        :param x: 输入张量
        :return: 输出张量
        """
        B, N, C, H, W = x.shape
        x = x.view(B * N, C, H, W)
        # Block 1
        x = self.pool(self.relu(self.bn1(self.conv1(x))))
        # Block 2
        x = self.pool(self.relu(self.bn2(self.conv2(x))))
        # Block 3
        x = self.pool(self.relu(self.bn3(self.conv3(x))))
        # 展平 + 全连接头
        x = self.flatten(x)
        x = self.dropout(self.relu(self.fc1(x)))
        x = self.dropout(self.relu(self.fc2(x)))
        x = self.dropout(self.relu(self.fc3(x)))
        x = self.fc4(x)
        x = x.view(B, N, -1)
        return x
        

class TrackFeatureExtractor(nn.Module):
    """
    轨迹特征提取器
    """
    def __init__(self, cfg) -> None:
        """
        构造函数
        :param cfg: 配置
        :return: None
        """
        super().__init__()
        """可能需要的参数"""
        self.cfg = cfg
        """特征预处理部分"""
        # 有可能需要在这里添加一些预处理
        """网络层"""
        # 全连接层
        self.fc1 = nn.LazyLinear(out_features=4)
        self.fc2 = nn.LazyLinear(out_features=64)
        self.fc3 = nn.LazyLinear(out_features=256)
        self.fc4 = nn.LazyLinear(out_features=64)
        self.fc5 = nn.LazyLinear(out_features=4)
        # 激活层
        self.relu = nn.ReLU()
        # dropout层
        self.dropout = nn.Dropout(p=0.1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        前向传播
        :param x: 输入张量
        :return: 输出张量
        """
        B, N, S = x.shape
        x = x.view(B * N, S)
        x = self.dropout(self.relu(self.fc1(x)))
        x = self.dropout(self.relu(self.fc2(x)))
        x = self.dropout(self.relu(self.fc3(x)))
        x = self.dropout(self.relu(self.fc4(x)))
        x = self.fc5(x)
        x = x.view(B, N, -1)
        return x


class PolicyNet(nn.Module):
    """
    策略网络
    """
    def __init__(self, cfg) -> None:
        """
        构造函数
        :param cfg: 配置
        :return: None
        """
        super().__init__()
        """可能需要的参数"""
        self.cfg = cfg
        self.device = cfg.device
        boundsNp = np.array(self.cfg.env.actionBounds, dtype=np.float32)
        assert boundsNp.shape[1] == 2, "actionBounds 每行需为[min,max]"
        self.actionLow = torch.tensor(boundsNp[:, 0], dtype=torch.float32, device=self.device)   # (4,)
        self.actionHigh = torch.tensor(boundsNp[:, 1], dtype=torch.float32, device=self.device)  # (4,)
        self.actionRange = self.actionHigh - self.actionLow
        # 供 logProb 常数修正 ( (high-low)/2 )
        self.logScaleConst = torch.log(self.actionRange / 2.0).sum()
        """特征提取器"""
        # 深度相机特征提取器
        self.depthCameraFeatureExtractor = DepthCameraFeatureExtractor(self.cfg)
        # 轨迹特征提取器
        self.trackFeatureExtractor = TrackFeatureExtractor(self.cfg)
        """网络层"""
        # 全连接层
        self.fc1 = nn.LazyLinear(out_features=64)
        self.fc2 = nn.LazyLinear(out_features=128)
        self.fc3 = nn.LazyLinear(out_features=256)
        self.fc4 = nn.LazyLinear(out_features=64)
        self.fcMu = nn.LazyLinear(out_features=4)  # 用于预测均值
        self.fcStd = nn.LazyLinear(out_features=4)  # 用于预测方差
        # 激活层
        self.relu = nn.ReLU()
        # dropout层
        self.dropout = nn.Dropout(p=0.1)
        # 放缩层
        self.scale = nn.Tanh()

    def forward(self, depthInformation: torch.Tensor, uavState: torch.Tensor) -> tuple:
        """
        前行传播
        :param depthInformation: 深度信息
        :param uavState: UAV状态
        :return: 动作和对数概率密度
        """
        """分别计算无人机的深度信息特征和无人机状态特征"""
        x1 = self.depthCameraFeatureExtractor(depthInformation)
        x2 = self.trackFeatureExtractor(uavState)
        """特征拼接"""
        x = torch.cat([x1, x2], dim=2)
        """策略网络"""
        x = self.dropout(self.relu(self.fc1(x)))
        x = self.dropout(self.relu(self.fc2(x)))
        x = self.dropout(self.relu(self.fc3(x)))
        x = self.dropout(self.relu(self.fc4(x)))
        mu = self.fcMu(x)  # 预测均值
        std = F.softplus(self.fcStd(x))  # 预测方差
        dist = Normal(mu, std)  # 正态分布
        normalSample = dist.rsample()  # 重参数化采样
        logProb = dist.log_prob(normalSample)  # 计算正态分布下的对数概率密度
        # tanh 压缩
        squashed = torch.tanh(normalSample)
        # tanh 的 Jacobian 修正
        logProb = logProb - torch.log(1 - squashed.pow(2) + 1e-7)
        action = self.actionLow + ( (squashed + 1.0) * 0.5 ) * self.actionRange
        # 线性缩放的对数概率再减常数项（不依赖 batch/N，每个样本相同，可广播）
        logProb = logProb - self.logScaleConst
        """计算tanh_normal分布的对数概率密度"""
        # logProb = logProb - torch.log(1 - torch.tanh(action).pow(2) + 1e-7)
        # action = action * self.actionBounds  # 放缩到动作空间
        return action, logProb
    

class QValueNet(nn.Module):
    """
    动作价值网络
    """
    def __init__(self, cfg) -> None:
        """
        构造函数
        :param cfg: 配置
        :return: None
        """
        super().__init__()
        """可能需要的参数"""
        self.cfg = cfg
        """特征提取器"""
        # 深度相机特征提取器
        self.depthCameraFeatureExtractor = DepthCameraFeatureExtractor(self.cfg)
        # 轨迹特征提取器
        self.trackFeatureExtractor = TrackFeatureExtractor(self.cfg)
        """网络层"""
        self.fc1 = nn.LazyLinear(out_features=64)
        self.fc2 = nn.LazyLinear(out_features=128)
        self.fc3 = nn.LazyLinear(out_features=256)
        self.fc4 = nn.LazyLinear(out_features=512)
        self.fc5 = nn.LazyLinear(out_features=256)
        self.fc6 = nn.LazyLinear(out_features=128)
        self.fc7 = nn.LazyLinear(out_features=64)
        self.fc8 = nn.LazyLinear(out_features=1)
        # 激活层
        self.relu = nn.ReLU()
        # dropout层
        self.dropout = nn.Dropout(p=0.1)

    def forward(self, depthInformation: torch.Tensor, uavState: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """
        前向传播
        :param depthInformation: 深度信息
        :param uavState: UAV状态
        :param action: 动作
        :return: Q值
        """
        """特征提取"""
        x1 = self.depthCameraFeatureExtractor(depthInformation)
        x2 = self.trackFeatureExtractor(uavState)
        """特征拼接"""
        x = torch.cat([x1, x2, action], dim=2)
        """Q值网络"""
        x = self.dropout(self.relu(self.fc1(x)))
        x = self.dropout(self.relu(self.fc2(x)))
        x = self.dropout(self.relu(self.fc3(x)))
        x = self.dropout(self.relu(self.fc4(x)))
        x = self.dropout(self.relu(self.fc5(x)))
        x = self.dropout(self.relu(self.fc6(x)))
        x = self.dropout(self.relu(self.fc7(x)))
        x = self.fc8(x)
        return x
    

class SAC:
    def __init__(self, cfg) -> None:
        """
        构造函数
        :param cfg: 配置
        :return: None
        """
        """可能需要的参数"""
        self.cfg = cfg
        self.actionDim = len(self.cfg.env.actionBounds)  # 动作维度
        """网络模型"""
        # 策略网络
        self.actor = PolicyNet(cfg).to(self.cfg.device)
        # 两个动作价值网络
        self.critic1 = QValueNet(cfg).to(self.cfg.device)
        self.critic2 = QValueNet(cfg).to(self.cfg.device)
        # 两个目标动作价值网络
        self.targetCritic1 = QValueNet(cfg).to(self.cfg.device)
        self.targetCritic2 = QValueNet(cfg).to(self.cfg.device)
        """模型参数相关"""
        self.targetCritic1.load_state_dict(self.critic1.state_dict())
        self.targetCritic2.load_state_dict(self.critic2.state_dict())
        # 优化器
        self.actorOptimizer = torch.optim.Adam(self.actor.parameters(), lr=self.cfg.actorLearningRate)
        self.criticOptimizer1 = torch.optim.Adam(self.critic1.parameters(), lr=self.cfg.criticLearningRate)
        self.criticOptimizer2 = torch.optim.Adam(self.critic2.parameters(), lr=self.cfg.criticLearningRate)
        # 学习率相关
        self.actorScheduler = torch.optim.lr_scheduler.StepLR(self.actorOptimizer, step_size=self.cfg.actorLearningRateDecayStep, gamma=self.cfg.actorLearningRateDecayGamma)
        self.criticScheduler1 = torch.optim.lr_scheduler.StepLR(self.criticOptimizer1, step_size=self.cfg.criticLearningRateDecayStep, gamma=self.cfg.criticLearningRateDecayGamma)
        self.criticScheduler2 = torch.optim.lr_scheduler.StepLR(self.criticOptimizer2, step_size=self.cfg.criticLearningRateDecayStep, gamma=self.cfg.criticLearningRateDecayGamma)
        # 使用alpha的log值，使得训练结果更稳定
        # self.logAlpha = torch.tensor(np.log(self.cfg.alpha), dtype=torch.float)
        # self.logAlpha.requires_grad = True  # 可以对其进行梯度计算
        self.logAlpha = torch.tensor(np.log(self.cfg.alpha), dtype=torch.float32, device=self.cfg.device, requires_grad=True)
        self.logAlphaOptimizer = torch.optim.Adam([self.logAlpha], lr=self.cfg.alphaLearningRate)  # 优化器
        self.targetEntropy = self.cfg.targetEntity  # 目标熵
        self.gamma = self.cfg.gamma  # 奖励折扣因子
        self.tau = self.cfg.tau  # soft update参数
        self.device = self.cfg.device  # 运行设备

    def take_action(self, uavInformations: tuple) -> list:
        """
        选择动作
        :param uavInformations: UAV信息，包含深度信息和UAV状态
        :return: 动作
        """
        # 过滤 None
        valid = [info for info in uavInformations if info is not None]
        if len(valid) == 0:
            return []
        # 提取状态
        depthInformation, uavState = self._classified_information(valid)
        # 检测数据中是否含有NaN
        if torch.isnan(depthInformation).any(): print("depth NaN")
        if torch.isnan(uavState).any(): print("state NaN")
        # 选择动作
        actions, _ = self.actor(depthInformation, uavState)
        if torch.isnan(actions).any() or torch.isinf(actions).any():
            print("[WARN] actions contain NaN/Inf, returning zeros")
            actions = torch.nan_to_num(actions, nan=0.0, posinf=0.0, neginf=0.0)
        return actions[0].detach().cpu().numpy()
    
    def calculate_target(self, rewards: int | float, depthInformation: torch.Tensor, uavState: torch.Tensor, dones: list) -> float | int:
        """
        计算目标Q值
        :param rewards: 奖励
        :param depthInformation: 深度信息
        :param uavState: UAV状态
        :param dones: 结束标志
        :return: td目标
        """
        with torch.no_grad():
            # 计算下一动作和概率
            nextActions, logProb = self.actor(depthInformation, uavState)
            # 熵
            entropy = -logProb
            # 获取两个动作价值网络输出
            q1Value = self.targetCritic1(depthInformation, uavState, nextActions)
            q2Value = self.targetCritic2(depthInformation, uavState, nextActions)
            # 取较小的Q值
            nextVaule = torch.min(q1Value, q2Value) + self.logAlpha.exp() * entropy
            # 计算时序差分目标
            tdTarget =rewards + self.gamma * nextVaule * (1 - dones)
        return tdTarget
    
    def soft_update(self, net, targetNet) -> None:
        """
        软更新
        :param net: 源网络
        :param targetNet: 目标网络
        :return: None
        """
        for paramTarget, param in zip(targetNet.parameters(), net.parameters()):
            paramTarget.data.copy_(paramTarget.data * (1.0 - self.tau) + param.data * self.tau)

    def update(self, transitionDict: dict):
        """
        更新参数模型
        :param transitionDict: 转换字典，包含状态、动作、奖励、下一个状态和结束标志
        :return: None
        """
        def _flt(lst):
            return [x for x in lst if x is not None]

        curStates     = _flt(transitionDict["states"])
        nextStates    = _flt(transitionDict["nextStates"])
        actions_raw   = _flt(transitionDict["actions"])
        rewards_raw   = _flt(transitionDict["rewards"])
        dones_raw     = _flt(transitionDict["dones"])
        if len(curStates) == 0 or len(nextStates) == 0:
            return
        # 状态张量
        depthInformation, uavState = self._classified_information(curStates)          # [1,N,C,H,W] / [1,N,S]
        nextDepthInformation, nextUavState = self._classified_information(nextStates)
        # 动作 (N, actionDim)
        actionsArr = np.array(actions_raw, dtype=np.float32)
        if actionsArr.ndim == 1:
            # 单动作维度情况
            if self.actionDim == 1:
                actionsArr = actionsArr.reshape(-1, 1)
            else:
                # 数据坏掉
                print("[WARN] actions shape invalid:", actionsArr.shape)
                return
        if actionsArr.shape[-1] != self.actionDim:
            print("[WARN] actions last dim mismatch:", actionsArr.shape, "expected", self.actionDim)
            return
        actions = torch.tensor(actionsArr, dtype=torch.float32, device=self.device).unsqueeze(0)  # [1,N,A]
        rewardsArr = np.array(rewards_raw, dtype=np.float32).reshape(1, -1, 1)
        donesArr   = np.array(dones_raw, dtype=np.float32).reshape(1, -1, 1)
        rewards = torch.tensor(rewardsArr, device=self.device)
        dones   = torch.tensor(donesArr, device=self.device)
        # 目标 Q
        tdTarget = self.calculate_target(rewards, nextDepthInformation, nextUavState, dones)
        # 当前 Q
        q1 = self.critic1(depthInformation, uavState, actions)
        q2 = self.critic2(depthInformation, uavState, actions)
        criticLoss1 = F.mse_loss(q1, tdTarget.detach())
        criticLoss2 = F.mse_loss(q2, tdTarget.detach())
        self.criticOptimizer1.zero_grad()
        criticLoss1.backward()
        torch.nn.utils.clip_grad_norm_(self.critic1.parameters(), 10.0)
        self.criticOptimizer1.step()
        self.criticOptimizer2.zero_grad()
        criticLoss2.backward()
        torch.nn.utils.clip_grad_norm_(self.critic2.parameters(), 10.0)
        self.criticOptimizer2.step()
        # 策略
        newActions, logProb = self.actor(depthInformation, uavState)  # [1,N,A], [1,N,A]
        if torch.isnan(newActions).any():
            print("[WARN] newActions NaN -> skip actor update")
            return
        entropy = -logProb
        q1_new = self.critic1(depthInformation, uavState, newActions)
        q2_new = self.critic2(depthInformation, uavState, newActions)
        actorLoss = torch.mean(-self.logAlpha.exp() * entropy - torch.min(q1_new, q2_new))
        self.actorOptimizer.zero_grad()
        actorLoss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 10.0)
        self.actorOptimizer.step()
        # alpha
        alphaLoss = torch.mean((entropy - self.targetEntropy).detach() * self.logAlpha.exp())
        self.logAlphaOptimizer.zero_grad()
        alphaLoss.backward()
        self.logAlphaOptimizer.step()
        # 软更新
        self.soft_update(self.critic1, self.targetCritic1)
        self.soft_update(self.critic2, self.targetCritic2)

    def _classified_information(self, uavInformations: tuple) -> tuple:
        """
        分类无人机信息
        :param uavInformations: UAV信息，包含深度信息和UAV状态
        :return: 深度信息，UAV状态
        depthInformation: [1,N,1,H,W]
        uavState:         [1,N,S]
        """
        depthList = []
        stateList = []
        for info in uavInformations:
            if info is None or not isinstance(info, dict):
                continue
            d = info.get("depthInformation", None)
            s = info.get("uavState", None)
            if d is not None:
                d = np.asarray(d, dtype=np.float32)
                d = np.nan_to_num(d, nan=0.0, posinf=0.0, neginf=0.0)
                depthList.append(d)
            if s is not None:
                s = np.asarray(s, dtype=np.float32)
                s = np.nan_to_num(s, nan=0.0, posinf=0.0, neginf=0.0)
                stateList.append(s)
        if len(depthList) == 0 or len(stateList) == 0:
            raise ValueError(f"_classified_information empty depth={len(depthList)} state={len(stateList)}")
        depthNp = np.stack(depthList, axis=0)  # [N,...]
        if depthNp.ndim == 3:
            # 补通道
            depthNp = depthNp[:, None, :, :]
        depthNp = np.nan_to_num(depthNp, nan=0.0, posinf=0.0, neginf=0.0)
        depthTensor = torch.tensor(depthNp, dtype=torch.float32, device=self.device).unsqueeze(0)  # [1,N,C,H,W]
        stateNp = np.stack(stateList, axis=0)
        stateNp = np.nan_to_num(stateNp, nan=0.0, posinf=0.0, neginf=0.0)
        stateTensor = torch.tensor(stateNp, dtype=torch.float32, device=self.device).unsqueeze(0)  # [1,N,S]
        if not torch.isfinite(depthTensor).all() or not torch.isfinite(stateTensor).all():
            print("[WARN] Non-finite in inputs -> sanitized")
            depthTensor = torch.nan_to_num(depthTensor, nan=0.0, posinf=0.0, neginf=0.0)
            stateTensor = torch.nan_to_num(stateTensor, nan=0.0, posinf=0.0, neginf=0.0)
        return depthTensor, stateTensor

if __name__ == "__main__":
    pass