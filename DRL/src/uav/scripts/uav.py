"""
无人机类
"""
from uav.scripts.multirotor_communication import Communication
import rospy
import threading
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, LaserScan
from dataclasses import dataclass
import numpy as np
from enum import Enum


@dataclass
class Position:
    """无人机位置"""
    x: float  # 无人机位置x坐标
    y: float  # 无人机位置y坐标
    z: float  # 无人机位置z坐标


class UAVResetState(Enum):
    """无人机重置状态枚举"""
    NORMAL = 0  # 正常状态
    RESETTING = 1  # 正在重置中
    RESET_COMPLETE = 2  # 重置完成


class UAVInfo(Enum):
    """无人机状态枚举"""
    SUCCESS = 0  # 成功到达目标点
    COLLISION = 1  # 发生碰撞
    STEP_OVER = 2  # 超出最大步长
    NORMAL = 3  # 正常状态


@dataclass
class SensorSpec:
    """传感器订阅规格"""
    topicTmpl: str  # 话题模板，使用{id}占位
    msgType: object  # ROS 消息类型
    callbackName: str  # UAV 实例上的回调方法名
    queueSize: int = 2  # 订阅队列大小


class UAV:

    # 传感器注册表：配置名->规格
    SENSOR_REGISTRY: dict[str, SensorSpec] ={
        "iris_realsense_camera": SensorSpec(
            topicTmpl="/iris_{id}/realsense/depth_camera/depth/image_raw",
            msgType=Image,
            callbackName="_depthImageCallback",
            queueSize=2,
        ),
        "iris_2d_lidar": SensorSpec(
            topicTmpl="/iris_{id}/scan",
            msgType=LaserScan,
            callbackName="_2dLidarCallback",
            queueSize=2,
        ),
    }

    def __init__(self, cfg, uavID: int| float) -> None:
        """
        构造函数
        :param cfg: 配置文件
        :param uavID: 无人机ID
        :return: None
        """
        """无人机信息"""
        self.cfg = cfg  # 配置文件
        self.uavID: int = uavID  # 无人机ID
        self.done: bool = False  # 终止状态
        self.info: UAVInfo = UAVInfo.NORMAL  # 无人机信息
        self.firstDone: bool = False  # 无人机是否第一次终止
        self.hoverFlage: bool = False # 无人机是否悬停好，悬停好后才可以开始飞行
        self.communication = Communication("iris", str(self.uavID))  # 无人机通信模块
        self.commThread = threading.Thread(target=self.communication.start)  # 通信线程
        self.commThread.daemon = True  # 守护线程
        self.commThread.start()  # 启动通信线程
        self.initPosition: Position = None  # 无人机起始位置
        self.currentPosition: Position = None  # 无人机当前位置（东北天坐标系下）
        self.currentYaw: float = None  # 无人机当前偏航角
        self.sensorData: np.ndarray = None  # 无人机当前传感器数据
        self.depthImage8u: np.ndarray = None  # 可视化用的8位灰度图
        """相关话题、服务"""
        # 短期目标发布
        self.shotTargetPub = rospy.Publisher("/xtdrone/"+"iris"+'_'+str(self.uavID)+"/cmd_pose_enu", Pose, queue_size=3)
        # 创建无人机传感器数据获取客户端
        self._setup_sensor_subscription()
    
    def getInformation(self) -> None:
        """
        获取无人机信息
        :return: None
        """
        self.currentPosition = (self.communication.current_position if self.initPosition is None 
                                else Position(self.communication.current_position.x + self.initPosition.x, 
                                              self.communication.current_position.y + self.initPosition.y, 
                                              self.communication.current_position.z + self.initPosition.z))
        self.currentYaw = self.communication.current_yaw

    def changeDone(self) -> None:
        """
        改变无人机终止状态
        :return: None
        """
        if self.hoverFlage:
            # 已经悬停好
            if self.done == False:
                # 第一次终止的标志位置位
                self.firstDone = True
            # 终止标志位置位
            self.done = True
        else:
            pass

    def _setup_sensor_subscription(self) -> None:
        """
        按配置从注册表创建传感器订阅
        :return: None
        """
        # 获取传感器类型
        sensorType = self.cfg.uav.sensorType
        sensor = self.SENSOR_REGISTRY.get(sensorType)  
        # 检查传感器是否存在
        if sensor is None:
            rospy.logerr(f"UAV {self.uavID}: Unknown sensor type '{sensorType}'")
            return
        # 格式化话题名称
        topicName = sensor.topicTmpl.format(id=self.uavID)
        # 获取回调方法
        callback = getattr(self, sensor.callbackName, None)
        if callback is None:
            rospy.logerr(f"UAV {self.uavID}: Unknown sensor callback '{sensor.callbackName}'")
            return
        # 创建传感器订阅
        self.sensorSub = rospy.Subscriber(topicName, sensor.msgType, callback, queue_size=sensor.queueSize)


    def _depthImageCallback(self, depthImage: Image) -> None:
        """
        深度相机数据回调：
        - 按 encoding 正确解析（32FC1: float32 米；16UC1: uint16 毫米）
        - 处理大小端与逐行对齐（step），裁剪到有效宽度
        - 将无效值（NaN/Inf 或 16UC1 的 0）统一为 NaN，再用“最远可见深度”填充，避免 NaN 传播
        - 生成两份数据：
            self.depthImage  -> float32（米），无 NaN，用于状态/奖励计算
            self.depthImage8u -> uint8（0-255），仅用于可视化
        """
        try:
            # 读取图像基础元数据
            imageHeight, imageWidth = int(depthImage.height), int(depthImage.width)
            encoding = depthImage.encoding                 # 常见: '32FC1' 或 '16UC1'
            isBigEndian = bool(depthImage.is_bigendian)    # 数据是否为大端序
            stepBytes = int(depthImage.step)               # 每行占用的总字节数（可能大于 width*bpp，含对齐填充）
            # 根据编码与大小端选择 numpy dtype 和每像素字节数
            if encoding.upper() == '32FC1':
                dataType = np.dtype('>f4' if isBigEndian else '<f4')  # 32 位浮点
                bytesPerPixel = 4
            elif encoding.upper() == '16UC1':
                dataType = np.dtype('>u2' if isBigEndian else '<u2')  # 16 位无符号整型
                bytesPerPixel = 2
            else:
                rospy.logwarn_throttle(5.0, f"Unsupported depth encoding: {encoding}")
                return
            # 使用 step 做逐行对齐读取，再裁剪到真实宽度（去掉行尾对齐填充）
            dataBuffer = memoryview(depthImage.data)               # 零拷贝视图
            flatArray = np.frombuffer(dataBuffer, dtype=dataType)  # 一维数组视图
            expectedElementsPerRow = stepBytes // bytesPerPixel    # 每行按 step 可容纳的元素数
            totalExpectedElements = expectedElementsPerRow * imageHeight
            if flatArray.size < totalExpectedElements:
                rospy.logwarn_throttle(5.0, f"Depth buffer too small: got {flatArray.size}, expected >= {totalExpectedElements}")
                return
            # 先按带对齐的行宽 reshape，再裁剪到 imageWidth，最后 copy 以脱离原始缓冲区
            depthArray = flatArray[:totalExpectedElements] \
                .reshape(imageHeight, expectedElementsPerRow)[:, :imageWidth] \
                .copy()
            # 将不同编码统一到 float32（米），并显式标注无效像素为 NaN
            if encoding.upper() == '16UC1':
                # RealSense 通常 16UC1 为毫米，且 0 表示无效
                invalidMask = (depthArray == 0)
                depthArray = depthArray.astype(np.float32)
                depthArray[invalidMask] = np.nan
                depthArray /= 1000.0  # mm -> m
            else:
                # 32FC1 通常已是米，但可能自带 NaN/Inf
                depthArray = depthArray.astype(np.float32, copy=False)
            # 将 Inf 也视为无效
            depthArray[~np.isfinite(depthArray)] = np.nan
            # 如果整帧都无效：保留上一帧，避免把全 NaN/全 0 注入状态与奖励
            finiteMask = np.isfinite(depthArray)
            if not np.any(finiteMask):
                if self.sensorData is not None:
                    rospy.logwarn_throttle(5.0, "Depth frame has no valid pixels; keeping previous depthImage")
                    return
                # 没有上一帧则安全回退为全 0（可按需求替换为相机最大量程）
                depthArray = np.zeros((imageHeight, imageWidth), dtype=np.float32)
            # 用当前帧的“有限像素最大值”填充 NaN（把无回波视为“最远”）
            if np.any(~finiteMask):
                finiteMax = np.nanmax(depthArray)
                depthArray = np.where(finiteMask, depthArray, finiteMax)
            # 保存用于训练/奖励的无 NaN 深度（米）
            self.sensorData = depthArray
            # 生成可视化 8-bit 灰度图：线性归一化到 [0,255]，并避免除零
            depthMin = float(np.min(depthArray))
            depthMax = float(np.max(depthArray))
            if depthMax > depthMin:
                normalized = (depthArray - depthMin) / (depthMax - depthMin)
            else:
                normalized = np.zeros_like(depthArray, dtype=np.float32)
            self.depthImage8u = (normalized * 255.0 + 0.5).astype(np.uint8)
        except Exception as error:
            # 节流打印避免刷屏
            rospy.logwarn_throttle(5.0, f"Depth callback error: {error}")


    def _2dLidarCallback(self, lidarScan: LaserScan) -> None:
        """
        激光雷达数据回调函数
        :param lidarScan: 激光雷达扫描数据
        :return: None
        """
        try:
            """获取2d激光雷达数据"""
            # 激光雷达数据范围
            rangeMin = lidarScan.range_min
            rangeMax = lidarScan.range_max
            # 激光雷达数据
            ranges = np.array(lidarScan.ranges)
            """处理2d激光雷达数据"""
            # 对inf的数据进行替换为rangeMax
            ranges = np.where(np.isinf(ranges), rangeMax, ranges)
            self.sensorData = ranges
        except Exception as error:
            # 节流打印避免刷屏
            rospy.logwarn_throttle(5.0, f"Lidar callback error: {error}")


if __name__ == "__main__":
    pass