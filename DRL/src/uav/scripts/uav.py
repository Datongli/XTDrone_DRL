"""
无人机类
"""
from uav.scripts.multirotor_communication import Communication
import rospy
import threading
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
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


class UAV:
    def __init__(self, uavID: int| float) -> None:
        """无人机信息"""
        self.uavID: int = uavID  # 无人机ID
        self.done: bool = False  # 终止状态
        self.firstDone: bool = False  # 无人机是否第一次终止
        self.hoverFlage: bool = False # 无人机是否悬停好，悬停好后才可以开始飞行
        self.communication = Communication("iris", str(self.uavID))  # 无人机通信模块
        self.commThread = threading.Thread(target=self.communication.start)  # 通信线程
        self.commThread.daemon = True  # 守护线程
        self.commThread.start()  # 启动通信线程
        self.initPosition: Position = None  # 无人机起始位置
        self.currentPosition: Position = None  # 无人机当前位置（东北天坐标系下）
        self.currentYaw: float = None  # 无人机当前偏航角
        self.depthImage: np.ndarray = None  # 无人机当前深度图像
        """相关话题、服务"""
        # 短期目标发布
        self.shotTargetPub = rospy.Publisher("/xtdrone/"+"iris"+'_'+str(self.uavID)+"/cmd_pose_enu", Pose, queue_size=3)
        # 无人机深度相机数据获取
        self.depthCameraSub = rospy.Subscriber("/iris_" + str(self.uavID) + "/realsense/depth_camera/depth/image_raw", Image, self._depthImageCallback ,queue_size=1)
    
    def getInformation(self) -> None:
        """获取无人机信息"""
        self.currentPosition = (self.communication.current_position if self.initPosition is None 
                                else Position(self.communication.current_position.x + self.initPosition.x, 
                                              self.communication.current_position.y + self.initPosition.y, 
                                              self.communication.current_position.z + self.initPosition.z))
        self.currentYaw = self.communication.current_yaw

    def changeDone(self) -> None:
        """改变无人机终止状态"""
        if self.hoverFlage:
            # 已经悬停好
            if self.done == False:
                # 第一次终止的标志位置位
                self.firstDone = True
            # 终止标志位置位
            self.done = True
        else:
            pass

    def _depthImageCallback(self, depthImage: Image) -> None:
        """深度相机数据回调函数"""
        data = bytes(depthImage.data)  # 转换为字节数组
        self.depthImage = np.frombuffer(data, dtype=np.float32).reshape((depthImage.height, depthImage.width))
        # 归一化到0-255
        minValue, maxValue = np.nanmin(self.depthImage), np.nanmax(self.depthImage)
        self.depthImage = ((self.depthImage - minValue) / (maxValue - minValue) * 255).astype(np.uint8)




if __name__ == "__main__":
    pass