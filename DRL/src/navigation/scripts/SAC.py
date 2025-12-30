"""
SAC算法主要代码
"""
import torch
import torch.nn.functional as F
import torch.nn as nn
from torch.distributions import Normal
import numpy as np
import copy
import math


class DepthCameraFeatureExtractor(nn.Module):
    """
    深度相机特征提取器
    """
    def __init__(self, cfg=None) -> None:
        """
        构造函数
        :param cfg: 配置
        :return: None
        """
        super().__init__()
        """可能需要的参数"""
        self.cfg = cfg
        """网络层"""
        # 卷积层
        self.conv1 = nn.LazyConv2d(out_channels=4, kernel_size=[4, 4], stride=[2, 2], padding=[1, 1])
        self.conv2 = nn.LazyConv2d(out_channels=16, kernel_size=[4, 4], stride=[2, 2], padding=[1, 1])
        self.conv3 = nn.LazyConv2d(out_channels=32, kernel_size=[4, 4], stride=[2, 2], padding=[1, 1])
        # 展平
        self.flatten = nn.Flatten()
        # 归一化层
        # self.bn1 = nn.LazyBatchNorm2d()
        # self.bn2 = nn.LazyBatchNorm2d()
        # self.bn3 = nn.LazyBatchNorm2d()
        self.bn1 = nn.Identity()  # 有说法是BN会在RL中引入不稳定噪声，因此暂时取消
        self.bn2 = nn.Identity()
        self.bn3 = nn.Identity()
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
        # self.dropout = nn.Dropout(p=0.1)
        self.dropout = nn.Identity()  # 有说法是dropout会在RL中引入不稳定噪声，因此暂时取消

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
        

# class LidarFeatureExtractor2D(nn.Module):
#     """
#     2D激光雷达特征提取器
#     """
#     def __init__(self, cfg=None) -> None:
#         """
#         构造函数
#         :param cfg: 配置
#         :return: None
#         """
#         super().__init__()
#         """可能需要的参数"""
#         self.cfg = cfg
#         """网络层"""
#         # 卷积层
#         self.conv1 = nn.LazyConv1d(out_channels=32, kernel_size=9, stride=2, padding=4)
#         self.conv2 = nn.LazyConv1d(out_channels=64, kernel_size=7, stride=2, padding=3)
#         self.conv3 = nn.LazyConv1d(out_channels=128, kernel_size=5, stride=2, padding=2)
#         self.conv4 = nn.LazyConv1d(out_channels=128, kernel_size=3, stride=1, padding=1)
#         # 激活层
#         self.relu = nn.ReLU()
#         # 归一化层
#         self.bn1 = nn.Identity()
#         self.bn2 = nn.Identity()
#         self.bn3 = nn.Identity()
#         self.bn4 = nn.Identity()
#         # 池化层
#         self.globalPool = nn.AdaptiveAvgPool1d(1)  # 全局平均池化
#         # 线性层
#         self.fc1 = nn.LazyLinear(out_features=256)
#         self.fc2 = nn.LazyLinear(out_features=128)
#         self.fc3 = nn.LazyLinear(out_features=64)
#         self.fc4 = nn.LazyLinear(out_features=64)
#         # dropout层
#         # self.dropout = nn.Dropout(p=0.5)
#         self.dropout = nn.Identity()

#     def forward(self, x: torch.Tensor) -> torch.Tensor:
#         """
#         前向传播
#         :param x: 输入张量
#         :return: 输出张量
#         """
#         B, N, C, L = x.shape
#         x = x.view(B * N, C, L)
#         x = self.relu(self.bn1(self.conv1(x)))
#         x = self.relu(self.bn2(self.conv2(x)))
#         x = self.relu(self.bn3(self.conv3(x)))
#         x = self.relu(self.bn4(self.conv4(x)))
#         x = self.globalPool(x).squeeze(-1)
#         x = self.dropout(self.relu(self.fc1(x)))
#         x = self.dropout(self.relu(self.fc2(x)))
#         x = self.dropout(self.relu(self.fc3(x)))
#         x = self.fc4(x)
#         x = x.view(B, N, -1)
#         return x


class LidarFeatureExtractor2D(nn.Module):
    """
    2D激光雷达特征提取器（512线 → 1D-CNN + Self-Attention）
    """
    def __init__(self, cfg=None) -> None:
        super().__init__()
        self.cfg = cfg
        # 1D-CNN：提取局部模式
        self.conv1 = nn.Conv1d(1, 16, kernel_size=5, stride=2, padding=2)
        self.conv2 = nn.Conv1d(16, 32, kernel_size=5, stride=2, padding=2)
        self.conv3 = nn.Conv1d(32, 64, kernel_size=5, stride=2, padding=2)
        self.bn1 = nn.BatchNorm1d(16) if getattr(cfg, "useBatchNorm", False) else nn.Identity()
        self.bn2 = nn.BatchNorm1d(32) if getattr(cfg, "useBatchNorm", False) else nn.Identity()
        self.bn3 = nn.BatchNorm1d(64) if getattr(cfg, "useBatchNorm", False) else nn.Identity()
        self.relu = nn.ReLU()
        self.pool = nn.MaxPool1d(2)
        # Self-Attention：捕捉全局依赖（简化版 Transformer Encoder Layer）
        self.attn = nn.MultiheadAttention(embed_dim=64, num_heads=4, batch_first=True)
        self.attn_norm = nn.LayerNorm(64)
        self.ffn = nn.Sequential(
            nn.Linear(64, 128),
            nn.ReLU(),
            nn.Linear(128, 64)
        )
        self.ffn_norm = nn.LayerNorm(64)
        # 全局池化 + 输出 MLP
        self.fc1 = nn.LazyLinear(128)
        self.fc2 = nn.LazyLinear(64)
        self.fc3 = nn.LazyLinear(4)
        self.dropout = nn.Identity()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        :param x: [B, N, 512]，B=batch, N=无人机数
        :return: [B, N, 4]
        """
        B, N, C, L = x.shape
        x = x.view(B * N, 1, L)  # [B*N, 1, 512]
        # 1D-CNN
        x = self.pool(self.relu(self.bn1(self.conv1(x))))  # -> [B*N, 16, 128]
        x = self.pool(self.relu(self.bn2(self.conv2(x))))  # -> [B*N, 32, 32]
        x = self.pool(self.relu(self.bn3(self.conv3(x))))  # -> [B*N, 64, 8]
        # Reshape for Attention: [B*N, SeqLen=8, Dim=64]
        x = x.permute(0, 2, 1)  # [B*N, 8, 64]
        # Self-Attention (residual)
        attn_out, _ = self.attn(x, x, x)
        x = self.attn_norm(x + attn_out)
        # FFN (residual)
        ffn_out = self.ffn(x)
        x = self.ffn_norm(x + ffn_out)  # [B*N, 8, 64]
        # 全局平均池化
        x = x.mean(dim=1)  # [B*N, 64]
        # MLP 输出
        x = self.dropout(self.relu(self.fc1(x)))
        x = self.dropout(self.relu(self.fc2(x)))
        x = self.fc3(x)  # [B*N, 4]
        x = x.view(B, N, -1)
        return x


# class TrackFeatureExtractor(nn.Module):
#     """
#     轨迹特征提取器
#     """
#     def __init__(self, cfg=None) -> None:
#         """
#         构造函数
#         :param cfg: 配置
#         :return: None
#         """
#         super().__init__()
#         """可能需要的参数"""
#         self.cfg = cfg
#         """网络层"""
#         # 全连接层
#         self.fc1 = nn.LazyLinear(out_features=4)
#         self.fc2 = nn.LazyLinear(out_features=64)
#         self.fc3 = nn.LazyLinear(out_features=256)
#         self.fc4 = nn.LazyLinear(out_features=64)
#         self.fc5 = nn.LazyLinear(out_features=4)
#         # 激活层
#         self.relu = nn.ReLU()
#         # dropout层
#         # self.dropout = nn.Dropout(p=0.1)
#         self.dropout = nn.Identity()  # 有说法是dropout会在RL中引入不稳定噪声，因此暂时取消

#     def forward(self, x: torch.Tensor) -> torch.Tensor:
#         """
#         前向传播
#         :param x: 输入张量
#         :return: 输出张量
#         """
#         B, N, S = x.shape
#         x = x.view(B * N, S)
#         x = self.dropout(self.relu(self.fc1(x)))
#         x = self.dropout(self.relu(self.fc2(x)))
#         x = self.dropout(self.relu(self.fc3(x)))
#         x = self.dropout(self.relu(self.fc4(x)))
#         x = self.fc5(x)
#         x = x.view(B, N, -1)
#         return x


class TrackFeatureExtractor(nn.Module):
    """
    轨迹特征提取器（状态：[dx, dy, dz, yaw]）
    """
    def __init__(self, cfg=None) -> None:
        super().__init__()
        self.cfg = cfg
        # 位置编码（可选，帮助网络理解周期性 yaw）
        self.fc1 = nn.LazyLinear(32)
        self.fc2 = nn.LazyLinear(64)
        self.fc3 = nn.LazyLinear(128)
        self.fc4 = nn.LazyLinear(64)
        self.fc5 = nn.LazyLinear(4)
        self.relu = nn.ReLU()
        self.dropout = nn.Identity()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        :param x: [B, N, 4]，[dx, dy, dz, yaw(已归一化到[-1,1])]
        :return: [B, N, 4]
        """
        B, N, S = x.shape
        x = x.view(B * N, S)
        # 对 yaw 做 sin/cos 编码（增强周期性理解）
        yaw = x[:, 3:4]  # [B*N, 1]
        yawSin = torch.sin(yaw * torch.pi)  # 已归一化到[-1,1]，乘π映射到[-π,π]
        yawCos = torch.cos(yaw * torch.pi)
        x = torch.cat([x[:, :3], yawSin, yawCos], dim=1)  # [B*N, 5]
        x = self.dropout(self.relu(self.fc1(x)))
        x = self.dropout(self.relu(self.fc2(x)))
        x = self.dropout(self.relu(self.fc3(x)))
        x = self.dropout(self.relu(self.fc4(x)))
        x = self.fc5(x)
        x = x.view(B, N, -1)
        return x


# 传感器提取器注册表
SENSOR_FEATURE_EXTRACTOR = {
        "iris_realsense_camera": DepthCameraFeatureExtractor(),
        "iris_2d_lidar": LidarFeatureExtractor2D(),
    }

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
        # self.logScaleConst = torch.log(self.actionRange / 2.0).sum()
        """特征提取器"""
        # 传感器特征提取器，深拷贝以避免共享参数
        baseExtractor = SENSOR_FEATURE_EXTRACTOR.get(self.cfg.uav.sensorType)
        if baseExtractor is None:
            raise ValueError(f"未知的传感器类型: {self.cfg.uav.sensorType}. 可用: {list(SENSOR_FEATURE_EXTRACTOR.keys())}")
        self.sensorFeatureExtractor = copy.deepcopy(baseExtractor)
        # 轨迹特征提取器
        self.trackFeatureExtractor = TrackFeatureExtractor(self.cfg)
        """网络层"""
        # 全连接层
        self.fc1 = nn.LazyLinear(out_features=64)
        self.fc2 = nn.LazyLinear(out_features=128)
        self.fc3 = nn.LazyLinear(out_features=256)
        self.fc4 = nn.LazyLinear(out_features=64)
        self.fcMu = nn.LazyLinear(out_features=4)  # 用于预测均值
        self.fcLogStd = nn.LazyLinear(out_features=4)  # 用于预测对数方差
        # 激活层
        self.relu = nn.ReLU()
        # dropout层
        # self.dropout = nn.Dropout(p=0.1)
        self.dropout = nn.Identity()  # 有说法是dropout会在RL中引入不稳定噪声，因此暂时取消
        # 放缩层
        self.scale = nn.Tanh()
        # 将常数项注册为 buffer，确保随 .to() 迁移设备
        self.register_buffer("logScaleConst", torch.log(self.actionRange / 2.0).sum())

    def forward(self, sensorData: torch.Tensor, uavState: torch.Tensor) -> tuple:
        """
        前行传播
        :param sensorData: 传感器数据
        :param uavState: UAV状态
        :return: 动作和对数概率密度
        """
        """分别计算无人机的传感器特征和无人机状态特征"""
        x1 = self.sensorFeatureExtractor(sensorData)
        x2 = self.trackFeatureExtractor(uavState)
        """特征拼接"""
        x = torch.cat([x1, x2], dim=2)
        """策略网络"""
        x = self.dropout(self.relu(self.fc1(x)))
        x = self.dropout(self.relu(self.fc2(x)))
        x = self.dropout(self.relu(self.fc3(x)))
        x = self.dropout(self.relu(self.fc4(x)))
        mu = self.fcMu(x)  # 预测均值 [B,N,A]
        # 约束对数方差，防止方差过小/过大
        logStd = torch.clamp(self.fcLogStd(x), min=-5.0, max=2.0)   # [B,N,A] 收紧下界，抑制过小方差
        std = torch.exp(logStd)
        # 非压缩高斯
        dist = Normal(mu, std)
        z = dist.rsample()                    # [B,N,A]
        logProb = dist.log_prob(z)            # [B,N,A]
        # tanh 压缩到 [-1,1]
        squashed = torch.tanh(z)              # [B,N,A]
        # tanh 的雅可比项：sum_over_A log(1 - tanh(z)^2)
        logDetJacobian = torch.log(1 - squashed.pow(2) + 1e-6)  # [B,N,A]
        # 在动作维上求和，得到标量对数概率 [B,N,1]
        logProb = (logProb - logDetJacobian).sum(dim=-1, keepdim=True)
        # 线性缩放到动作空间
        action = self.actionLow + ((squashed + 1.0) * 0.5) * self.actionRange  # [B,N,A]
        # 数值稳健：按边界再夹紧一次（支持逐维 Tensor 边界的广播）
        # 线性缩放的对数概率修正：减去 sum log((high-low)/2)
        action = torch.max(torch.min(action, self.actionHigh), self.actionLow)
        logProb = logProb - self.logScaleConst  # [B,N,1]
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
        # 传感器特征提取器，深拷贝以避免共享参数
        baseExtractor = SENSOR_FEATURE_EXTRACTOR.get(self.cfg.uav.sensorType)
        if baseExtractor is None:
            raise ValueError(f"未知的传感器类型: {self.cfg.uav.sensorType}. 可用: {list(SENSOR_FEATURE_EXTRACTOR.keys())}")
        self.sensorFeatureExtractor = copy.deepcopy(baseExtractor)
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
        # self.dropout = nn.Dropout(p=0.1)
        self.dropout = nn.Identity()  # 有说法是dropout会在RL中引入不稳定噪声，因此暂时取消
        # 把动作边界注册为 buffer，便于将动作归一化到[-1,1]
        boundsNp = np.array(self.cfg.env.actionBounds, dtype=np.float32)  # 形如 [[-1,1],...,[ -pi, pi ]]
        actionLow = torch.tensor(boundsNp[:, 0], dtype=torch.float32)
        actionHigh = torch.tensor(boundsNp[:, 1], dtype=torch.float32)
        actionRange = actionHigh - actionLow
        self.register_buffer("qActionLow", actionLow)         # [A]
        self.register_buffer("qActionRange", actionRange)     # [A]

    def forward(self, sensorData: torch.Tensor, uavState: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """
        前向传播
        :param sensorData: 传感器数据
        :param uavState: UAV状态
        :param action: 动作
        :return: Q值
        """
        """特征提取"""
        x1 = self.sensorFeatureExtractor(sensorData)
        x2 = self.trackFeatureExtractor(uavState)
        # 新增：将动作缩放到[-1,1]，消除各维量纲差异（尤其 yaw）
        # scaled = 2*(a - low)/range - 1
        safeRange = torch.clamp(self.qActionRange, min=1e-6)
        scaledAction = 2.0 * (action - self.qActionLow) / safeRange - 1.0
        scaledAction = torch.clamp(scaledAction, -1.0, 1.0)
        """特征拼接"""
        x = torch.cat([x1, x2, scaledAction], dim=2)
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
        self.networks = {
            "actor": self.actor,
            "critic1": self.critic1,
            "critic2": self.critic2,
            "targetCritic1": self.targetCritic1,
            "targetCritic2": self.targetCritic2,
        }
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
        self.logAlpha = torch.tensor(np.log(self.cfg.alpha), dtype=torch.float32, device=self.cfg.device, requires_grad=True)
        self.logAlphaOptimizer = torch.optim.Adam([self.logAlpha], lr=self.cfg.alphaLearningRate)  # 优化器
        self.targetEntropy = float(getattr(self.cfg, "targetEntropy", -float(self.actionDim)))
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
        sensorData, uavState = self._classified_information(valid)
        # 选择动作
        actions, _ = self.actor(sensorData, uavState)
        if torch.isnan(actions).any() or torch.isinf(actions).any():
            print("[WARN] actions contain NaN/Inf, returning zeros")
            actions = torch.nan_to_num(actions, nan=0.0, posinf=0.0, neginf=0.0)
        return actions[0].detach().cpu().numpy()
    
    def calculate_target(self, rewards: int | float, sensorData: torch.Tensor, uavState: torch.Tensor, dones: list) -> float | int:
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
            nextActions, logProb = self.actor(sensorData, uavState)
            # 熵
            entropy = -logProb
            # 获取两个动作价值网络输出
            q1Value = self.targetCritic1(sensorData, uavState, nextActions)
            q2Value = self.targetCritic2(sensorData, uavState, nextActions)
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
        # 成对过滤，保持索引一致
        paired = []
        for s, a, r, ns, d in zip(transitionDict["states"],
                                  transitionDict["actions"],
                                  transitionDict["rewards"],
                                  transitionDict["nextStates"],
                                  transitionDict["dones"]):
            if s is None or a is None or r is None or ns is None or d is None:
                continue
            paired.append((s, a, r, ns, d))
        if not paired:
            return
        curStates, actionsRaw, rewardsRaw, nextStates, dones_raw = map(list, zip(*paired))
        # 状态张量
        depthInformation, uavState = self._classified_information(curStates)          # [1,N,C,H,W] / [1,N,S]
        nextDepthInformation, nextUavState = self._classified_information(nextStates)
        # 动作 (N, actionDim)
        actionsArr = np.array(actionsRaw, dtype=np.float32)
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
        rewardsArr = np.array(rewardsRaw, dtype=np.float32).reshape(1, -1, 1)
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
        actorLoss = (self.logAlpha.exp() * logProb - torch.min(q1_new, q2_new)).mean()
        lambdaA = float(getattr(self.cfg, "actorActionL2", 1e-3))
        actorLoss = actorLoss + lambdaA * (newActions.pow(2).mean())
        self.actorOptimizer.zero_grad()
        actorLoss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 5.0)
        self.actorOptimizer.step()
        # alpha
        alphaLoss = -(self.logAlpha * (logProb + self.targetEntropy).detach()).mean()
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
        :return: 传感器数据，UAV状态
        深度相机 -> [1, N, 1, H, W]
        2D雷达   -> [1, N, 1, L]
        UAV状态 -> [1, N, S]
        """
        # 1) 成对过滤，确保每个条目同时具有传感器数据与UAV状态，保证索引对齐
        pairedList = []
        for informationItem in uavInformations:
            if not informationItem or not isinstance(informationItem, dict):
                continue
            sensorDataRaw = informationItem.get("sensorData", None)  
            uavStateRaw = informationItem.get("uavState", None)
            if sensorDataRaw is None or uavStateRaw is None:
                continue
            pairedList.append((sensorDataRaw, uavStateRaw))
        if not pairedList:
            raise ValueError("_classified_information: empty inputs")
        sensorDataList, uavStateList = map(list, zip(*pairedList))
        # 2) 传感器数据预处理（自动或按配置识别雷达/相机）
        # 先转为对象数组以保留各自形状便于探测
        sensorObjectArray = np.asarray(sensorDataList, dtype=object)
        # 将每个元素都标准化为 float32 的 ndarray
        sensorNumpyList = []
        for arrayItem in sensorObjectArray:
            arrayItem = np.asarray(arrayItem, dtype=np.float32)
            sensorNumpyList.append(arrayItem)
        # 尝试直接堆叠；若形状不一致（如有些是 [L] 有些是 [1,L]），做一次修正后再堆叠
        try:
            sensorNumpyArray = np.stack(sensorNumpyList, axis=0)
        except Exception:
            fixedList = []
            for arrayItem in sensorNumpyList:
                # 兼容常见雷达形状 [1,L] 或 [L,1] -> 展平成 [L]
                if arrayItem.ndim == 2 and (arrayItem.shape[0] == 1 or arrayItem.shape[1] == 1):
                    arrayItem = arrayItem.reshape(-1)
                fixedList.append(arrayItem)
            sensorNumpyArray = np.stack(fixedList, axis=0)
        # 按配置优先判断传感器类型；未配置时再用形状启发式判定
        sensorTypeString = str(getattr(getattr(self.cfg, "uav", None), "sensorType", "")).lower()
        isLidar = ("lidar" in sensorTypeString)
        if not isLidar:
            # 形状启发：
            # - 雷达常见形状: [N, L] 或 [N, 1, L]
            # - 相机常见形状: [N, H, W] 或 [N, C, H, W]
            if sensorNumpyArray.ndim == 2:
                isLidar = True
            elif sensorNumpyArray.ndim == 3 and sensorNumpyArray.shape[1] == 1 and sensorNumpyArray.shape[-1] >= 16:
                isLidar = True
            else:
                isLidar = False
        if isLidar:
            # 2D 雷达分支
            # 目标张量形状: [1, N, 1, L]
            # 步骤:
            # - 统一形状到 [N, 1, L]
            # - 将无效值（NaN/Inf）替换为物理合理范围
            # - 使用[minLidarMeters, maxLidarMeters]将距离线性缩放到[0,1]
            if sensorNumpyArray.ndim == 1:
                sensorNumpyArray = sensorNumpyArray[None, ...]  # [N, L]
            if sensorNumpyArray.ndim == 2:
                sensorNumpyArray = sensorNumpyArray[:, None, :]  # [N, 1, L]
            elif sensorNumpyArray.ndim == 3:
                # 若形如 [N, L, 1] 则转成 [N, 1, L]
                if sensorNumpyArray.shape[1] != 1 and sensorNumpyArray.shape[2] == 1:
                    sensorNumpyArray = sensorNumpyArray[:, :, 0][:, None, :]
                elif sensorNumpyArray.shape[1] != 1 and sensorNumpyArray.shape[2] != 1:
                    raise ValueError(f"Unexpected lidar shape: {sensorNumpyArray.shape}")
            minRangeMeters = float(getattr(self.cfg.env, "minLidarMeters",
                                           getattr(self.cfg.env, "minDepthMeters", 0.5)))
            maxRangeMeters = float(getattr(self.cfg.env, "maxLidarMeters",
                                           getattr(self.cfg.env, "maxDepthMeters", 100.0)))
            # 替换无效值 -> 裁剪到量程 -> 线性缩放到[0,1]
            sensorNumpyArray = np.nan_to_num(sensorNumpyArray,
                                             nan=maxRangeMeters, posinf=maxRangeMeters, neginf=minRangeMeters)
            sensorNumpyArray = np.clip(sensorNumpyArray, minRangeMeters, maxRangeMeters)
            sensorNumpyArray = (sensorNumpyArray - minRangeMeters) / (maxRangeMeters - minRangeMeters + 1e-6)
            sensorTensor = torch.tensor(sensorNumpyArray, dtype=torch.float32, device=self.device).unsqueeze(0)
        else:
            # 深度相机分支
            # 目标张量形状: [1, N, 1, H, W]（若已有多通道则为 [1, N, C, H, W]）
            # 步骤:
            # - 若为 [N, H, W]，补通道为 [N, 1, H, W]
            # - 将无效值（NaN/Inf）替换为物理合理范围
            # - 使用[minDepthMeters, maxDepthMeters]将深度线性缩放到[0,1]
            if sensorNumpyArray.ndim == 3:
                sensorNumpyArray = sensorNumpyArray[:, None, :, :]  # [N, 1, H, W]
            elif sensorNumpyArray.ndim != 4:
                raise ValueError(f"Unexpected camera shape: {sensorNumpyArray.shape}")
            minDepthMeters = float(getattr(self.cfg.env, "minDepthMeters", 0.1))
            maxDepthMeters = float(getattr(self.cfg.env, "maxDepthMeters", 50.0))
            sensorNumpyArray = np.nan_to_num(sensorNumpyArray,
                                             nan=maxDepthMeters, posinf=maxDepthMeters, neginf=minDepthMeters)
            sensorNumpyArray = np.clip(sensorNumpyArray, minDepthMeters, maxDepthMeters)
            sensorNumpyArray = (sensorNumpyArray - minDepthMeters) / (maxDepthMeters - minDepthMeters + 1e-6)
            sensorTensor = torch.tensor(sensorNumpyArray, dtype=torch.float32, device=self.device).unsqueeze(0)
        # 3) UAV 状态预处理
        # 目标张量形状: [1, N, S]
        # 步骤:
        # - 堆叠为 [N, S]
        # - 若启用 normalizeState:
        #   * 将偏航角包裹到[-pi, pi]
        #   * 按环境尺度 [length, width, height, pi] 逐维缩放到[-1,1]
        #   * 若状态维度 > 4，额外维度用1作为缩放（避免除零/未知量纲）
        stateNumpyArray = np.stack([np.asarray(uavState, dtype=np.float32) for uavState in uavStateList], axis=0)
        if getattr(self.cfg, "normalizeState", True):
            if stateNumpyArray.shape[1] >= 4:
                stateNumpyArray[:, 3] = self._wrap_to_pi(stateNumpyArray[:, 3])
            defaultMaxAbsArray = np.array([
                float(getattr(self.cfg.env, "length", 100.0)),
                float(getattr(self.cfg.env, "width", 100.0)),
                float(getattr(self.cfg.env, "height", 50.0)),
                np.pi
            ], dtype=np.float32)
            if stateNumpyArray.shape[1] > 4:
                extraScaleArray = np.ones((stateNumpyArray.shape[1] - 4,), dtype=np.float32)
                maxAbsArray = np.concatenate([defaultMaxAbsArray, extraScaleArray], axis=0)
            else:
                maxAbsArray = defaultMaxAbsArray[: stateNumpyArray.shape[1]]
            maxAbsArray = np.clip(maxAbsArray, 1e-6, np.inf)
            stateNumpyArray = stateNumpyArray / maxAbsArray
            stateNumpyArray = np.clip(stateNumpyArray, -1.0, 1.0)
        stateNumpyArray = np.nan_to_num(stateNumpyArray, nan=0.0, posinf=0.0, neginf=0.0)
        stateTensor = torch.tensor(stateNumpyArray, dtype=torch.float32, device=self.device).unsqueeze(0)
        # 4) 数值安全校验：如仍存在非有限数，做兜底替换
        if not torch.isfinite(sensorTensor).all() or not torch.isfinite(stateTensor).all():
            print("[WARN] Non-finite in inputs -> sanitized")
            sensorTensor = torch.nan_to_num(sensorTensor, nan=0.0, posinf=0.0, neginf=0.0)
            stateTensor = torch.nan_to_num(stateTensor, nan=0.0, posinf=0.0, neginf=0.0)
        return sensorTensor, stateTensor
    
    def _wrap_to_pi(self, angleArray: np.ndarray) -> np.ndarray:
        """
        将角度数组包装到[-pi,pi]范围内
        :param angleArray: 角度数组
        :return: 包装后的角度数组
        """
        return ((angleArray + np.pi) % (2.0 * np.pi)) - np.pi

if __name__ == "__main__":
    pass