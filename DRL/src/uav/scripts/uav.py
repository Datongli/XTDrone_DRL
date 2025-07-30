"""
无人机类
"""
from uav.scripts.multirotor_communication import Communication
import rospy
import threading
from geometry_msgs.msg import Pose


class UAV:
    def __init__(self, uavID: int| float) -> None:
        """无人机信息"""
        self.uavID = uavID  # 无人机ID
        self.done = False  # 终止状态
        self.communication = Communication("iris", str(self.uavID))  # 无人机通信模块
        self.commThread = threading.Thread(target=self.communication.start)  # 通信线程
        self.commThread.daemon = True  # 守护线程
        self.commThread.start()  # 启动通信线程
        self.x: float  # 无人机x坐标
        self.y: float  # 无人机y坐标
        self.z: float  # 无人机z坐标
        self.roll: float  # 无人机横滚角
        self.pitch: float  # 无人机俯仰角
        self.yaw: float  # 无人机偏航角
        """相关话题、服务"""
        # 短期目标发布
        self.shotTargetPub = rospy.Publisher("/xtdrone/"+"iris"+'_'+str(self.uavID)+"/cmd_pose_enu", Pose, queue_size=3)

    def getInformation(self) -> None:
        """获取无人机信息"""
        pass

    def setPosition(self, x: float, y: float, z: float) -> None:
        """设置无人机位置"""
        self.x = x
        self.y = y
        self.z = z




if __name__ == "__main__":
    pass