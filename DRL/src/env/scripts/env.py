"""
环境类
"""
import gymnasium as gym
import subprocess
import rospy
from std_srvs.srv import Empty
import math
import numpy as np
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
from env.scripts.target import Target
from env.scripts.obstacle import Obstacle
from visualization_msgs.msg import MarkerArray, Marker
import os


class StaticObstacleEnv(gym.Env):
    """静态障碍物环境"""
    def __init__(self, cfg) -> None:
        """初始化环境中参数"""
        # 长宽高
        self.length: int| float = cfg.env.length
        self.width: int| float = cfg.env.width
        self.height: int| float = cfg.env.height
        self.staticObstaclesNum: int = cfg.env.staticObstaclesNum  # 静态障碍物数量
        self.staticObstacles: list[Obstacle] = []  # 静态障碍物集合
        self.obstacleHorizontalRange: int| float = cfg.env.obstacleHorizontalRange  # 障碍物水平半径
        self.obstacleVerticalRange: int| float = cfg.env.obstacleVerticalRange  # 障碍物垂直高度
        self.uavNums: int = cfg.uav.uavNums  # 无人机数量
        self.targets: list[Target] = []  # 不同无人机的目标点集合
        self.targetRadius: int| float = cfg.env.targetRadius  # 目标点半径
        port = "11311"  # ROS端口号
        """使用给定的启动文件名来启动模拟"""
        # 调整gazebo世界world文件的参数
        self._world_adaption()
        # ROS节点初始化
        rospy.init_node("static_obstacle_env", anonymous=True)
        # 启动launch文件
        launchFile = cfg.launchFile
        if launchFile.startswith("/"):
            fullpath = launchFile
        else:
            fullpath = os.path.join(os.path.dirname(os.path.dirname(__file__)), "launch", launchFile)
        if not os.path.exists(fullpath):
            raise FileNotFoundError(f"文件不存在: {fullpath}")
        # 启动launch文件
        subprocess.Popen(["roslaunch", "-p", port, fullpath])
        rospy.loginfo("launch文件启动!")
        """设置topic发布者、订阅者、服务"""
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)          # 恢复物理模拟
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)              # 暂停物理模拟
        self.resetProxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)           # 重置世界服务
        self.deleteModel = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)   # 删除模型服务
        self.spawnModel = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)  # 生成模型服务
        self.targetVisualization = rospy.Publisher("/target", MarkerArray, queue_size=3)  # 目标点可视化
        
    def reset(self):
        """
        重置环境
        重新随机生成障碍物，将无人机重置到初始位置并悬停，规划无人机目标点
        发布目标点和无人机状态，传感器数据
        """
        super().reset()
        """重置gazebo环境"""
        # 世界重置服务
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.resetProxy()
        except rospy.ServiceException as e:
            rospy.logerr("reset环境失败: %s", e)
        # 删除旧障碍物
        rospy.wait_for_service("/gazebo/delete_model")
        for i in range(self.staticObstaclesNum):
            try:
                self.deleteModel("cube_" + str(i))
            except rospy.ServiceException as e:
                if "does not exist" in str(e):
                    pass  # 模型不存在时忽略
                else:
                    rospy.logerr("删除模型失败: %s", e)
        # 无人机归位，悬停
        self._uav_reset()
        """生成无人机目标点"""
        self._target_generate()
        """生成静态障碍物"""
        self._static_obstacles_generate()
        

    def step(self):
        """
        一步模拟
        """
        pass


    def render(self):
        pass

    
    def _world_adaption(self):
        """
        根据cfg中的参数，调整gazebo世界world文件的参数
        主要作用是根据cfg中的参数，调整gazebo世界中四个围墙的位置
        """
        with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "launch/XTDrone_DRL.world"), 'w') as f:
            with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "launch/XTDrone_DRL.world.template"), 'r') as template:
                content = template.read()
                content = content.format(
                    envLength=self.length,
                    envWidth=self.width,
                    envHeight=self.height,
                    envLengthHalf=self.length / 2.0,
                    envWidthHalf=self.width / 2.0,
                    envHeightHalf=self.height / 2.0,
                    lengthDeviationFront=0.0,
                    widthDeviationFront=-self.width / 2.0,
                    lengthDeviationLeft=-self.length / 2.0,
                    widthDeviationLeft=0.0,
                    lengthDeviationRight=self.length / 2.0,
                    widthDeviationRight=0.0,
                    lengthDeviationBack=0.0,
                    widthDeviationBack=self.width / 2.0
                )
                f.write(content)

    def _uav_reset(self):
        """
        无人机归位，悬停
        """
        pass

    def _target_generate(self) -> None:
        """
        生成目标点
        """
        # 每个无人机生成一个目标点
        for i in range(self.uavNums):
            while True:
                flag = True
                # 随机生成目标点
                x = np.random.uniform(self.length * 0.2, self.length * 0.8)
                y = np.random.uniform(self.width * 0.2, self.width * 0.8)
                z = np.random.uniform(self.height * 0.5, self.height * 0.95)
                # 判断与其他目标点的距离是否过近
                for otherTarget in self.targets:
                    if math.sqrt((x - otherTarget.x) ** 2 + (y - otherTarget.y) ** 2 + (z - otherTarget.z) ** 2) < self.targetRadius * 4:
                        flag = False
                        break
                if flag:
                    break
            self.targets.append(Target(i, x, y, z))


    def _static_obstacles_generate(self) -> None:
        """
        生成静态障碍物
        """
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        # 读取sdf模板文件
        with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "models/cube_template.sdf"), 'r') as f:
            cubeTemplate = f.read()
        minDistBetweenObstacles = self.length * 0.04  # 障碍物之间最小距离
        minDistBetweenObstaclesAndTargets = self.targetRadius * 4  # 障碍物与目标点之间最小距离
        for i in range(self.staticObstaclesNum):
            for _ in range(100):  # 最多尝试100次
                flag = True
                """随机生成障碍物大小和位置"""
                # 随机生成障碍物大小
                halfLength = np.random.uniform(self.obstacleHorizontalRange * 0.2, self.obstacleHorizontalRange)
                halfWidth = np.random.uniform(self.obstacleHorizontalRange * 0.2, self.obstacleHorizontalRange)
                height = np.random.uniform(self.obstacleVerticalRange * 0.4, self.obstacleVerticalRange)
                # 随机生成障碍物位置
                x = np.random.uniform(self.length * 0.1, self.length * 0.9)
                y = np.random.uniform(self.width * 0.1, self.width * 0.9)
                z = height / 2.0  # 确保障碍物底面在地面上
                # 检查与其他障碍物的距离
                for otherObstacle in self.staticObstacles:
                    if math.sqrt((x - otherObstacle.x) ** 2 + (y - otherObstacle.y) ** 2 + (z - otherObstacle.z) ** 2) < minDistBetweenObstacles:
                        flag = False
                        break
                # 检查与目标点的距离
                for target in self.targets:
                    if math.sqrt((x - target.x) ** 2 + (y - target.y) ** 2 + (z - target.z) ** 2) < minDistBetweenObstaclesAndTargets:
                        flag = False
                        break
                if flag:
                    break
            self.staticObstacles.append(Obstacle(x, y, z, halfLength, halfWidth, height))
            """用模版生成SDF"""
            cubeSDF = cubeTemplate.format(size_x=halfLength * 2, size_y=halfWidth * 2, size_z=height)
            pose = Pose(position=Point(x, y, z), orientation=Quaternion(0, 0, 0, 1))
            try:
                self.spawnModel(
                    model_name = "cube_" + str(i),
                    model_xml = cubeSDF,
                    robot_namespace = "/",
                    initial_pose = pose,
                    reference_frame = "world"
                )
            except rospy.ServiceException as e:
                rospy.logerr("生成障碍物失败: %s", e)

    def _publish_maker(self):
        """
        消息发布
        """
        """发布目标点，用于rviz中可视化"""
        markerArray = MarkerArray()
        for target in self.targets:
            marker = Marker()
            marker.header.frame_id = "world"  # 目标点坐标系
            marker.type = marker.SPHERE  # 渲染成球体
            marker.action = marker.ADD  # 增加
            marker.id = target.uavID
            # 目标点大小
            marker.scale.x = self.targetRadius * 2
            marker.scale.y = self.targetRadius * 2
            marker.scale.z = self.targetRadius * 2
            # 渲染的颜色和不透明度
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0  # 无旋转
            # 目标点位置
            marker.pose.position.x = target.x
            marker.pose.position.y = target.y
            marker.pose.position.z = target.z
            markerArray.markers.append(marker)
        self.targetVisualization.publish(markerArray)




if __name__ == "__main__":
    pass