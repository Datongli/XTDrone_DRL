"""
环境类
"""
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
import gymnasium as gym
import subprocess
import rospy
from std_srvs.srv import Empty
import math
import numpy as np
import tf
from gazebo_msgs.srv import SpawnModel, DeleteModel
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, TwistStamped
from env.scripts.target import Target
from env.scripts.obstacle import Obstacle
from visualization_msgs.msg import MarkerArray, Marker
from gazebo_msgs.msg import ModelState, ContactsState
from uav.scripts.uav import UAV, Position, UAVResetState, UAVInfo
from mavros_msgs.srv import ParamSet, CommandLong, ParamGet
from mavros_msgs.msg import ParamValue
import threading
import time


class StaticObstacleEnv(gym.Env):
    """
    静态障碍物环境
    """
    def __init__(self, cfg) -> None:
        """
        构造函数
        :param cfg: 配置
        :return: None
        """
        self.cfg = cfg
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
        self.stepCount: int = 0  # 步数
        port = "11311"  # ROS端口号
        """使用给定的启动文件名来启动模拟"""
        # 调整gazebo世界world文件的参数
        self._world_adaption()
        # ROS节点初始化
        rospy.init_node("static_obstacle_env", anonymous=True)
        # 按照要求修改launch文件
        launchFile = cfg.launchFile
        if launchFile.startswith("/"):
            fullpath = launchFile
        else:
            fullpath = os.path.join(os.path.dirname(os.path.dirname(__file__)), "launch", launchFile)
        if not os.path.exists(fullpath):
            raise FileNotFoundError(f"文件不存在: {fullpath}")
        # 生成多无人机launch文件
        self._generate_multi_uav_launch(fullpath)
        # 启动launch文件
        subprocess.Popen(["roslaunch", "-p", port, fullpath])
        rospy.loginfo("launch文件启动!")
        # 等待 Gazebo 服务就绪
        rospy.loginfo("等待 Gazebo 服务启动...")
        self._wait_for_gazebo_services()
        """环境中的无人机集合"""
        self.uavs: list[UAV] = [UAV(cfg, i) for i in range(self.uavNums)]
        """添加异步重置相关属性"""
        self.uavResetStates = [UAVResetState.NORMAL for _ in range(self.uavNums)]  # 无人机重置状态
        self.resetThreads = [None for _ in range(self.uavNums)]  # 重置线程
        self.resetLock = threading.Lock()  # 重置操作锁
        self.activeResetCount: int =0  # 进行中的重置计数
        """设置topic发布者、订阅者、服务"""
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)          # 恢复物理模拟
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)              # 暂停物理模拟
        self.resetProxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)           # 重置世界服务
        self.deleteModel = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)   # 删除模型服务
        self.spawnModel = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)  # 生成模型服务
        self.targetVisualization = rospy.Publisher("/target", MarkerArray, queue_size=3)  # 目标点可视化
        self.uavSetState  = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=10)  # 无人机位置和姿态初始化设置
        # 无人机碰撞检测
        self.collisionDetectionSub = rospy.Subscriber("/benchmarker/collision", ContactsState, self._collisionDetectionCallback, queue_size=2)

    def reset(self) -> tuple:
        """
        重置环境
        重新随机生成障碍物，将无人机重置到初始位置并悬停，规划无人机目标点
        发布目标点和无人机状态，传感器数据
        :return: 无人机信息
        """
        super().reset()
        """gazebo物理仿真继续运行"""
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except rospy.ServiceException as e:
            rospy.logerr("取消暂停gazebo物理仿真失败: %s", e)
        # 等待所有无人机完成异步重置
        self._wait_for_all_uavs_reset_complete()
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
        # 重置所有无人机状态为正常
        with self.resetLock:
            for i in range(self.uavNums):
                self.uavResetStates[i] = UAVResetState.NORMAL
                self.uavs[i].done = False
                self.uavs[i].firstDone = False
        # 无人机归位，悬停
        self._uav_reset()
        """生成无人机目标点"""
        self._target_generate()
        """生成静态障碍物"""
        self._static_obstacles_generate()
        rospy.loginfo("环境重置完成，开始进行无人机信息采集...")
        """获取无人机和环境信息，暂停gazebo仿真"""
        # 无人机信息采集
        uavInformations = []
        for uav in self.uavs:
            sensorData, uavState = self._uav_information_collection(uav)  # 传感器数据和无人机状态
            uavInformations.append({"sensorData": sensorData, 
                                    "uavState": uavState,
                                    "uavZ": uav.currentPosition.z})
        rospy.loginfo("无人机信息采集完成")
        # 暂停gazebo物理仿真
        try:
            self.pause()
        except rospy.ServiceException as e:
            rospy.logerr("暂停gazebo物理仿真失败: %s", e)
        return uavInformations
        
    def step(self, actions: np.ndarray) -> tuple:
        """
        一步模拟，并行执行
        :param actions: 无人机的动作，每个无人机的动作包括：下一秒的三维动作，航向角
                        n个，n的数量不定，只给done=flase的无人机进行动作
        :return: 无人机的状态，奖励
        """
        """欲返回信息"""
        nextStates = [None for _ in range(self.uavNums)]  # 无人机的状态
        rewards = [0 for _ in range(self.uavNums)]  # 奖励
        dones = [False for _ in range(self.uavNums)]  # 每个无人机是否结束
        """gazebo物理仿真继续运行"""
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except rospy.ServiceException as e:
            rospy.logerr("取消暂停gazebo物理仿真失败: %s", e)
        """执行无人机动作"""
        rate = rospy.Rate(50)  # 50Hz
        for _ in range(100):
            i = 0  # 计时一段时间
            for uav in self.uavs:
                # 跳过已完成或正在重置的无人机
                if uav.firstDone or uav.done or self.uavResetStates[uav.uavID] != UAVResetState.NORMAL:
                    continue
                pose = self.make_pose(
                    uav.currentPosition.x - uav.initPosition.x + actions[i][0],
                    uav.currentPosition.y - uav.initPosition.y + actions[i][1],
                    uav.currentPosition.z - uav.initPosition.z + actions[i][2],
                    actions[i][3])
                uav.shotTargetPub.publish(pose)
                i += 1  # 无人机索引增加
            rate.sleep()
        """判断无人机状态与计算奖励"""
        self.stepCount += 1  # 步数增加
        for i, uav in enumerate(self.uavs):
            # 跳过正在重置的无人机
            if self.uavResetStates[i] == UAVResetState.RESETTING:
                nextStates[i] = None
                rewards[i] = None
                continue
            # 状态为done且不是第一次done的无人机
            if uav.done and not uav.firstDone:
               nextStates[i] = None
               rewards[i] = None
               continue
            # 获取无人机信息
            sensorData, uavState = self._uav_information_collection(uav)  # 传感器数据和无人机状态
            if self.cfg.uav.sensorType == "iris_realsense_camera":
                # 深度相机中障碍物的距离惩罚
                if sensorData is not None:
                    rewards[i] += -1 / (np.min(sensorData) + 1e-3) * 2
            elif self.cfg.uav.sensorType == "iris_2d_lidar":
                # 激光传感器中障碍物的距离惩罚
                if sensorData is not None:
                    rewards[i] += -1 / (np.min(sensorData) + 1e-3) * 2
            # 碰撞惩罚
            if uav.currentPosition.z >= self.height: 
                uav.changeDone()
            # 超过最大步长
            if self.stepCount == self.cfg.env.maxStep:
                uav.changeDone()
                uav.info = UAVInfo.STEP_OVER  # 将无人机信息设置为超出最大步长
            if uav.firstDone:  # 此时firstDone为True的是发生了碰撞或者超出最大步长的
                rewards[i] -= 1000
                if uav.info == UAVInfo.NORMAL:
                    uav.info = UAVInfo.COLLISION  # 将无人机信息设置为碰撞
            # 靠近目标奖励
            rewards[i] += - np.sqrt((self.targets[i].x - uav.currentPosition.x) ** 2 + 
                                    (self.targets[i].y - uav.currentPosition.y) ** 2 + 
                                    (self.targets[i].z - uav.currentPosition.z) ** 2) * 5
            # 到达目标奖励
            if np.sqrt((self.targets[i].x - uav.currentPosition.x) ** 2 + 
                       (self.targets[i].y - uav.currentPosition.y) ** 2 + 
                       (self.targets[i].z - uav.currentPosition.z) ** 2) <= self.targetRadius:
                rewards[i] += 1000
                uav.info = UAVInfo.SUCCESS  # 将无人机信息设置为成功
                uav.changeDone()
            nextStates[i] = {"sensorData": sensorData, 
                             "uavState": uavState,
                             "uavZ": uav.currentPosition.z}
            dones[i] = uav.done
            # 将第一次done的无人机，归位，锁定，本轮不再起飞
            if uav.firstDone:
                # 启动异步重置
                self._start_async_uav_reset(uav)
        """暂停gazebo物理仿真"""
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            # 若任有进行中的无人机异步重置，跳过暂停
            if self.activeResetCount == 0:
                self.unpause()
            else:
                rospy.loginfo("仍有 %d 个无人机正在重置，跳过暂停gazebo物理仿真", self.activeResetCount)
        except rospy.ServiceException as e:
            rospy.logerr("暂停gazebo物理仿真失败: %s", e)
        return nextStates, rewards, dones

    def render(self) -> None:
        """
        gymnasium规定的渲染函数，在当前环境中并不使用，但是需要重写
        :return: None
        """
        pass

    def _collisionDetectionCallback(self, collisionData: ContactsState) -> None:
        """
        碰撞检测回调函数
        :param collisionData: 碰撞数据
        :return: None
        """
        uavID = ["iris_" + str(i) for i in range(self.uavNums)]
        for contact in collisionData.states:
            if contact.collision1_name[:6] in uavID:
                self.uavs[int(contact.collision1_name[5])].changeDone()
            if contact.collision2_name[:6] in uavID:
                self.uavs[int(contact.collision2_name[5])].changeDone()

    def _world_adaption(self) -> None:
        """
        根据cfg中的参数，调整gazebo世界world文件的参数
        主要作用是根据cfg中的参数，调整gazebo世界中四个围墙的位置
        :return: None
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

    def _generate_multi_uav_launch(self, launchFile: str) -> None:
        """
        生成多无人机launch文件
        :param launchFile: launch文件路径
        :return: None
        """
        try:
            """构造多无人机launch文件"""
            # launch文件头
            header = '''
            <launch>
            <!--控制Gazobo UI是否开启的参数-->
            <arg name="gui" value="true"/>

                <!--控制Gazebo开启-->
                <include file="$(find env)/launch/empty_world.launch"/>
            '''
            groupTQL = '''
            <!--iris_{i}-->
            <group ns="iris_{i}">
                <!--MAVROS配置-->
                    <arg name="ID" value="{i}"/>
                    <arg name="ID_in_group" value="{i}"/>
                    <arg name="fcu_url" default="udp://:{udpPort}@localhost:{gcsPort}"/>
                <!--PX4 SITL以及无人机产生-->
                <include file="$(find px4)/launch/single_vehicle_spawn_xtd.launch">
                    <arg name="x" value="{x}"/>
                    <arg name="y" value="{y}"/>
                    <arg name="z" value="{z}"/>
                    <arg name="R" value="{R}"/>
                    <arg name="P" value="{P}"/>
                    <arg name="Y" value="{Y}"/>
                    <arg name="vehicle" value="iris"/>
                    <arg name="sdf" value="{sensorType}"/>
                    <arg name="mavlink_udp_port" value="{mavlinkUdpPort}"/>
                    <arg name="mavlink_tcp_port" value="{mavlinkTcpPort}"/>
                    <arg name="ID" value="$(arg ID)"/>
                    <arg name="ID_in_group" value="$(arg ID_in_group)"/>
                </include>
                <!--MAVROS-->
                <include file="$(find mavros)/launch/px4.launch">
                    <arg name="fcu_url" value="$(arg fcu_url)"/>
                    <arg name="gcs_url" value=""/>
                    <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
                    <arg name="tgt_component" value="1"/>
                </include>
            </group>
            '''
            footer = '</launch>\n'
            with open(launchFile, 'w') as f:
                f.write(header)
                for i in range(self.uavNums):
                    f.write(groupTQL.format(
                        i=i,
                        udpPort=24540+i,
                        gcsPort=34580+i,
                        x=self.length // self.uavNums * i + self.length / (2 * self.uavNums),
                        y=self.cfg.uav.initPosition.y,
                        z=0.5,
                        R=0.0,
                        P=0.0,
                        Y=math.pi/2,
                        sensorType=self.cfg.uav.sensorType,
                        mavlinkUdpPort=18570+i,
                        mavlinkTcpPort=4560+i,
                    ))
                f.write(footer)
        except Exception as e:
            print(f"生成多无人机launch文件失败: {e}")

    def _wait_for_gazebo_services(self) -> None:
        """
        等待 Gazebo 服务启动
        :return: None
        """
        servicesToWait = [
            "/gazebo/unpause_physics",
            "/gazebo/pause_physics", 
            "/gazebo/reset_world",
            "/gazebo/delete_model",
            "/gazebo/spawn_sdf_model"
        ]
        for serviceName in servicesToWait:
            rospy.loginfo(f"等待服务: {serviceName}")
            try:
                rospy.wait_for_service(serviceName, timeout=30)  # 30秒超时
                rospy.loginfo(f"服务 {serviceName} 已就绪")
            except rospy.ROSException:
                rospy.logerr(f"等待服务 {serviceName} 超时!")
                raise
        rospy.loginfo("所有 Gazebo 服务已就绪")
        # 等待 MAVROS 节点启动
        self._wait_for_mavros_nodes()

    def _wait_for_mavros_nodes(self) -> None:
        """
        等待所有 MAVROS 节点启动
        :return: None
        """
        rospy.loginfo("等待 MAVROS 节点启动...")
        for i in range(self.uavNums):
            # 等待每个无人机的 MAVROS 节点
            mavros_state_topic = f"/iris_{i}/mavros/state"
            rospy.loginfo(f"等待 MAVROS 节点: {mavros_state_topic}")
            # 使用简单的计数器而不是 rospy.Time
            max_attempts = 60  # 30秒 (每次等待0.5秒)
            attempts = 0
            while attempts < max_attempts:
                try:
                    # 尝试获取话题列表
                    topics = rospy.get_published_topics()
                    topic_names = [topic[0] for topic in topics]
                    if mavros_state_topic in topic_names:
                        rospy.loginfo(f"MAVROS 节点 iris_{i} 已启动")
                        break
                except Exception as e:
                    rospy.logdebug(f"获取话题列表失败: {e}")
                rospy.sleep(0.5)
                attempts += 1
            else:
                raise rospy.ROSException(f"等待 MAVROS 节点 iris_{i} 超时!")
        # 额外等待，确保 MAVROS 完全初始化
        rospy.loginfo("等待 MAVROS 完全初始化...")
        rospy.sleep(5)
        rospy.loginfo("所有 MAVROS 节点已就绪")

    def _wait_for_all_uavs_reset_complete(self) -> None:
        """
        等待所有无人机完成异步重置
        :return: None
        """
        rospy.loginfo("等待所有无人机完成异步重置...")
        maxWaitTime = 60  # 最大等待时间（秒）
        startTime = time.time()
        while time.time() - startTime < maxWaitTime:
            with self.resetLock:
                allComplete = True
                for i, state in enumerate(self.uavResetStates):
                    if state == UAVResetState.RESETTING:
                        allComplete = False
                        rospy.loginfo(f"等待无人机 iris_{i} 完成重置...")
                        break
                if allComplete:
                    rospy.loginfo("所有无人机异步重置完成")
                    return
            time.sleep(1.0)  # 每秒检查一次
        # 超时处理
        rospy.logwarn("等待无人机重置超时，强制继续")
        with self.resetLock:
            for i in range(self.uavNums):
                if self.uavResetStates[i] == UAVResetState.RESETTING:
                    self.uavResetStates[i] = UAVResetState.RESET_COMPLETE
                    rospy.logwarn(f"强制标记无人机 iris_{i} 重置完成")

    def _uav_reset(self) -> None:
        """
        所有无人机归位并悬停（并行批量处理）
        :return: None
        """
        """归位所有无人机"""
        for uav in self.uavs:
            self._send_flight_termination(uav, False)
        stateMsgs = []
        uavTargetPositions = []  # 记录每个无人机的目标位置
        rate = rospy.Rate(50)  # 50Hz
        for uav in self.uavs:
            # 改变无人机done状态
            uav.hoverFlage = False
            stateMsg = ModelState()
            stateMsg.model_name = "iris_" + str(uav.uavID)
            # 计算初始位置
            initialX = self.length // self.uavNums * uav.uavID + self.length / (2 * self.uavNums)
            initialY = self.cfg.uav.initPosition.y
            initialZ = self.cfg.uav.initPosition.z
            stateMsg.pose.position.x = initialX
            stateMsg.pose.position.y = initialY
            stateMsg.pose.position.z = initialZ
            uav.initPosition = Position(initialX, initialY, initialZ-0.25)
            # 计算目标悬停位置（在初始位置上方5米）
            targetX = self.cfg.uav.hoverPosition.x
            targetY = self.cfg.uav.hoverPosition.y
            targetZ = self.cfg.uav.hoverPosition.z
            uavTargetPositions.append((targetX, targetY, targetZ))
            angle = np.pi / 2  # 90度，面朝y轴正方向
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
            stateMsg.pose.orientation.x = quaternion[0]
            stateMsg.pose.orientation.y = quaternion[1]
            stateMsg.pose.orientation.z = quaternion[2]
            stateMsg.pose.orientation.w = quaternion[3]
            stateMsgs.append(stateMsg)
            self.uavSetState.publish(stateMsg)
        rospy.loginfo("无人机归位完成，开始悬停...")
        # 延长发布时间，确保PX4接收到足够的setpoint
        for i in range(50):  # 发布1秒
            for j, uav in enumerate(self.uavs):
                pose = self.make_pose(uavTargetPositions[j][0],
                                       uavTargetPositions[j][1],
                                       uavTargetPositions[j][2], 
                                       np.pi/2)
                uav.shotTargetPub.publish(pose)
            rate.sleep()
        # 等待无人机位置更新
        rospy.sleep(2)
        # 确保所有无人机的通信都已建立
        rospy.loginfo("检查无人机通信状态...")
        for i, uav in enumerate(self.uavs):
            while uav.communication.current_position is None:
                rospy.logwarn(f"等待无人机 {i} 位置信息...")
                rospy.sleep(0.5)
            rospy.loginfo(f"无人机 {i} 通信正常")
        """启动并悬停无人机，使用绝对目标位置"""
        rospy.loginfo("开始发布目标位置...")
        # 延长发布时间，确保PX4接收到足够的setpoint
        for i in range(500):  # 发布
            for j, uav in enumerate(self.uavs):
                pose = self.make_pose(uavTargetPositions[j][0],
                                       uavTargetPositions[j][1],
                                       uavTargetPositions[j][2], 
                                       np.pi/2)
                uav.shotTargetPub.publish(pose)
            rate.sleep()
        # 检查所有无人机EKF状态（可选，至少要有current_position）
        for i, uav in enumerate(self.uavs):
            retry = 0
            while (uav.communication.current_position is None or
                abs(getattr(uav.communication, "current_velocity", 0)) > 0.5):
                rospy.logwarn(f"等待无人机 {i} EKF收敛...")
                rospy.sleep(0.5)
                retry += 1
                if retry > 20:
                    break
        rospy.loginfo("切换到OFFBOARD模式...")
        # 逐个切换无人机模式，增加延时
        for i, uav in enumerate(self.uavs):
            rospy.loginfo(f"切换无人机 {i} 到OFFBOARD模式")
            cmdMsg = String()
            cmdMsg.data = "OFFBOARD"
            uav.communication.cmd_callback(cmdMsg)
            rospy.sleep(3.0)  # 增加等待时间到3秒
            cmdMsg.data = "ARM"
            uav.communication.cmd_callback(cmdMsg)
            rospy.loginfo(f"解锁无人机 {i}")
            rospy.sleep(2.0)  # 增加等待时间到2秒
        # 继续发布setpoint，确保无人机起飞
        rospy.loginfo("持续发布setpoint，等待起飞...")
        for i in range(300):  # 继续发布
            for j, uav in enumerate(self.uavs):
                pose = self.make_pose(uavTargetPositions[j][0],
                                       uavTargetPositions[j][1],
                                       uavTargetPositions[j][2], 
                                       np.pi/2)
                uav.shotTargetPub.publish(pose)
            rate.sleep()
        # 检查无人机是否到达指定高度，增加超时机制
        rospy.loginfo("检查无人机起飞状态...")
        timeout_count = 0
        max_timeout = 600  # 30秒超时
        while timeout_count < max_timeout:
            readyNum = 0
            for uav in self.uavs:
                if uav.communication.current_position and uav.communication.current_position.z > 4.5:
                    readyNum += 1
            if readyNum == self.uavNums:
                rospy.loginfo("所有无人机已到达目标高度")
                break
            # 继续发布setpoint
            for j, uav in enumerate(self.uavs):
                pose = self.make_pose(uavTargetPositions[j][0],
                                       uavTargetPositions[j][1],
                                       uavTargetPositions[j][2], 
                                       np.pi/2)
                uav.shotTargetPub.publish(pose)
            rate.sleep()
            timeout_count += 1
        rospy.loginfo(f"共{self.uavNums}个无人机，已达到目标高度有{readyNum}个")
        if timeout_count >= max_timeout:
            rospy.logwarn("部分无人机未能到达目标高度，继续执行...")
        # 切换到悬停状态，为了观察而设置的，如果后面紧跟其他状态（step），需要注释掉
        rospy.loginfo("切换到悬停状态...")
        for uav in self.uavs:
            cmdMsg = String()
            cmdMsg.data = "HOVER"
            uav.communication.cmd_callback(cmdMsg)
            uav.hoverFlage = True  # 悬停标志位置位
            uav.done = False  # done标志位置位
            uav.firstDone = False  # firstDone标志位置位
        # 等待无人机稳定
        rospy.sleep(2)

    @staticmethod
    def make_pose(x, y, z, yaw) -> Pose:
        """
        静态方法，用于创建Pose对象
        :param x: x坐标
        :param y: y坐标
        :param z: z坐标
        :param yaw: 偏航角度
        :return: Pose对象
        """
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def _target_generate(self) -> None:
        """
        生成目标点
        :return: None
        """
        # 先清空目标点
        self.targets.clear()
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
            self.targets.append(Target(x, y, z))

    def _static_obstacles_generate(self) -> None:
        """
        生成静态障碍物
        :return: None
        """
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        # 读取sdf模板文件
        with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "models/cube_template.sdf"), 'r') as f:
            cubeTemplate = f.read()
        minDistBetweenObstacles = self.length * 0.04  # 障碍物之间最小距离
        minDistBetweenObstaclesAndTargets = self.targetRadius * 2  # 障碍物与目标点之间最小距离
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
                    if math.sqrt((x - target.x) ** 2 + (y - target.y) ** 2 + (z - target.z) ** 2) < (minDistBetweenObstaclesAndTargets + np.sqrt(halfLength ** 2 + halfWidth ** 2)):
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

    def _uav_information_collection(self, uav: UAV) -> tuple:
        """
        无人机信息采集
        :param uav: 无人机对象
        :return: 无人机深度信息和状态
        """
        # 采集无人机信息
        uav.getInformation()
        # 传感器数据，无人机状态
        sensorData = uav.sensorData
        uavState = [self.targets[uav.uavID].x - uav.currentPosition.x, 
                    self.targets[uav.uavID].y - uav.currentPosition.y, 
                    self.targets[uav.uavID].z - uav.currentPosition.z,
                    uav.currentYaw]
        return sensorData, uavState

    def _send_flight_termination(self, uav: UAV, enable: bool) -> None:
        """
        发送 flight termination 命令:
        enable=True 立即终止；enable=False 解除终止
        :param uav: 无人机对象
        :param enable: 是否启用飞行终止
        :return: None
        """
        try:
            cmdService = rospy.ServiceProxy(f"/iris_{uav.uavID}/mavros/cmd/command", CommandLong)
            resp = cmdService(
                broadcast=False,
                command=185,  # MAV_CMD_DO_FLIGHTTERMINATION
                confirmation=0,
                param1=1 if enable else 0, 
                param2=0, param3=0, param4=0, param5=0, param6=0, param7=0
            )
            rospy.loginfo(f"flight termination {'ON' if enable else 'OFF'} iris_{uav.uavID}, success={resp.success}")
        except Exception as e:
            rospy.logerr(f"终止飞行命令失败：iris_{uav.uavID}, {e}")

    def _start_async_uav_reset(self, uav: UAV) -> None:
        """
        启动无人机异步重置
        :param uav: 无人机对象
        :return: None
        """
        with self.resetLock:
            if self.uavResetStates[uav.uavID] != UAVResetState.NORMAL:
                return  # 已在重置中，避免重复启动
            self.uavResetStates[uav.uavID] = UAVResetState.RESETTING
            self.activeResetCount += 1  # 增加进行中的重置计数
            # 启动重置线程
            resetThread = threading.Thread(
                target = self._async_uav_reset_worker,
                args=(uav,),
                daemon=True
            )
            self.resetThreads[uav.uavID] = resetThread
            resetThread.start()
            rospy.loginfo(f"开始异步重置无人机：iris_{uav.uavID}")

    def _async_uav_reset_worker(self, uav: UAV) -> None:
        """
        异步无人机重置工作线程
        :param uav: 无人机对象
        :return: None
        """
        try:
            rospy.loginfo(f"异步重置无人机 iris_{uav.uavID} 开始")
            # 取消gazebo暂停
            try:
                self.unpause()
            except rospy.ServiceException as e:
                rospy.logerr("取消gazebo暂停失败: %s", e)
            # 重置firstDone标志
            uav.firstDone = False
            # 发送飞行终止命令
            self._send_flight_termination(uav, True)
            rospy.sleep(0.1)
            # Gazebo模型归位
            stateMsg = ModelState()
            stateMsg.model_name = "iris_" + str(uav.uavID)
            stateMsg.pose = self.make_pose(uav.initPosition.x, uav.initPosition.y, uav.initPosition.z, np.pi / 2)
            stateMsg.twist.linear.x, stateMsg.twist.linear.y, stateMsg.twist.linear.z = 0, 0, 0
            stateMsg.twist.angular.x, stateMsg.twist.angular.y, stateMsg.twist.angular.z = 0, 0, 0
            self.uavSetState.publish(stateMsg)
            rospy.loginfo(f"Gazebo模型重置完成：iris_{uav.uavID}")
            # disarm
            self._force_disarm(uav)
            rospy.sleep(0.1)
            # 等待gazebo物理仿真暂停（如果正在运行的话）
            # 注意：这里不直接暂停，因为其他无人机可能还在运行
            # 清理PX4/IMU（参数、EKF），并喂入初始状态
            self._comprehensive_px4_reset(uav)
            # 主动向EKF提供新的初始位姿，帮助收敛
            self._provide_initial_state(uav)
            # 设置无人机状态
            uav.done = True
            uav.getInformation()
            # rospy.loginfo(f"无人机 iris_{uav.uavID} 的记录坐标为x={uav.currentPosition.x}, y={uav.currentPosition.y}, z={uav.currentPosition.z}")
            rospy.loginfo(f"无人机 iris_{uav.uavID} 已锁定在地面上")
            # 让无人机降落
            cmdMsg = String()
            cmdMsg.data = "AUTO.LAND"
            uav.communication.cmd_callback(cmdMsg)
            rospy.sleep(5)  # 等待降落完成
            # 标记重置完成
            with self.resetLock:
                self.uavResetStates[uav.uavID] = UAVResetState.RESET_COMPLETE
            rospy.loginfo(f"异步重置无人机 iris_{uav.uavID} 完成")
        except Exception as e:
            rospy.logerr(f"异步重置无人机 iris_{uav.uavID} 失败: {e}")
            # 即使失败也标记为完成，避免死锁
            with self.resetLock:
                self.uavResetStates[uav.uavID] = UAVResetState.RESET_COMPLETE
        finally:
            with self.resetLock:
                self.activeResetCount = max(0, self.activeResetCount - 1)

    def _emergency_motor_kill(self, uav: UAV) -> None:
        """
        立即终止无人机电机：螺旋桨停止
        :param uav: 无人机对象
        :return: None
        """
        try:
            cmdService = rospy.ServiceProxy("/iris_" + str(uav.uavID) + "/mavros/cmd/command", CommandLong)
            resp = cmdService(
                broadcast=False,
                command=185,  # MAV_CMD_DO_FLIGHTTERMINATION
                confirmation=0,
                param1=1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0
            )
            rospy.loginfo(f"终止飞行命令：iris_{uav.uavID}, success={resp.success}")
        except Exception as e:
            rospy.logerr(f"终止飞行命令失败：iris_{uav.uavID}, {e}")

    def _force_disarm(self, uav: UAV) -> None:
        """
        显式DISARM保证控制器复位
        :param uav: 无人机对象
        :return: None
        """
        try:
            cmdService = rospy.ServiceProxy(f"/iris_{uav.uavID}/mavros/cmd/command", CommandLong)
            # 首先DISARM
            resp = cmdService(
                broadcast=False, 
                command=400,  # MAV_CMD_COMPONENT_ARM_DISARM
                confirmation=0,
                param1=0,     # disarm
                param2=0, param3=0, param4=0, param5=0, param6=0, param7=0
            )
            rospy.loginfo(f"DISARM命令：iris_{uav.uavID}, success={resp.success}")
            rospy.sleep(0.5)
        except Exception as e:
            rospy.logerr(f"强制DISARM/ARM失败：iris_{uav.uavID}, {e}")

    def _comprehensive_px4_reset(self, uav: UAV) -> None:
        """
        重置PX4关键参数，避免状态混乱
        :param uav: 无人机对象
        :return: None
        """
        rospy.loginfo(f"开始综合重置PX4状态：iris_{uav.uavID}")
        try:
            paramService = rospy.ServiceProxy(f'/iris_{uav.uavID}/mavros/param/set', ParamSet)
            cmdService = rospy.ServiceProxy(f'/iris_{uav.uavID}/mavros/cmd/command', CommandLong)
            # 重置关键参数
            criticalParams = {
                "EKF2_AID_MASK": 1,      # 使用GPS/基础融合
                "EKF2_HGT_MODE": 0,      # 气压计高度
                "EKF2_GPS_CHECK": 0,     # 禁用GPS检查（SITL方便）
                "COM_RCL_EXCEPT": 4,     # 允许OFFBOARD
                "COM_OBL_ACT": 0,        # OFFBOARD丢失着陆
                # 安全检查禁用（仅用于仿真）
                "COM_ARM_WO_GPS": 1,        # 允许无GPS解锁
                "CBRK_IO_SAFETY": 22027,    # 禁用安全开关
                # 传感器相关
                "SYS_HAS_MAG": 0,           # 禁用磁力计要求
            }
            # 获取当前PX4版本的有效参数列表（可选）
            validParams = self._get_valid_px4_params(uav)
            for paramName, paramVal in criticalParams.items():
                # 如果获取了有效参数列表，检查参数是否存在
                if validParams and paramName not in validParams:
                    rospy.logwarn(f"跳过不存在的参数：{paramName}")
                    continue   
                try:
                    paramValue = ParamValue()
                    # 根据参数名推断数据类型
                    if isinstance(paramVal, float) or "ALT" in paramName or "ANG" in paramName:
                        paramValue.real = float(paramVal)
                    else:
                        paramValue.integer = int(paramVal)  
                    response = paramService(param_id=paramName, value=paramValue)
                    if response.success:
                        rospy.loginfo(f"重置参数成功 {paramName} = {paramVal}")
                    else:
                        rospy.logwarn(f"重置参数失败 {paramName}")
                except Exception as e:
                    rospy.logwarn(f"设置参数 {paramName} 异常: {e}")
            # 增加等待时间让参数生效
            rospy.sleep(3.0)
            # 发送重新校准命令
            self._send_recalibration_commands(uav, cmdService)
            rospy.loginfo(f"PX4状态重置完成: iris_{uav.uavID}")
        except Exception as e:
            rospy.logerr(f"综合重置PX4状态异常：iris_{uav.uavID}, {e}")

    def _get_valid_px4_params(self, uav: UAV) -> set:
        """
        获取当前PX4版本支持的参数列表
        :param uav: 无人机对象
        :return: 支持的参数名集合
        """
        try:
            paramGetService = rospy.ServiceProxy(f'/iris_{uav.uavID}/mavros/param/get', ParamGet)
            # 尝试获取一个已知参数来测试服务
            validParams = set()
            # 可以通过获取参数列表或者预定义已知参数
            knownParams = [
                "EKF2_AID_MASK", "EKF2_HGT_MODE", "EKF2_GPS_CHECK",
                "COM_RCL_EXCEPT", "COM_OBL_ACT", "COM_ARM_WO_GPS",
                "CBRK_IO_SAFETY", "SYS_HAS_MAG"
            ]
            for param in knownParams:
                try:
                    response = paramGetService(param_id=param)
                    if response.success:
                        validParams.add(param)
                except:
                    pass
            return validParams
        except Exception as e:
            rospy.logwarn(f"无法获取参数列表: {e}")
            return None
        
    def _send_recalibration_commands(self, uav: UAV, cmdService: rospy.ServiceProxy) -> None:
        """
        发送重新校准命令
        :param uav: 无人机对象
        :param cmdService: 命令服务代理
        :return: None
        """
        try:
            # 重置EKF
            response = cmdService(
                broadcast=False,
                command=252,  # MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
                confirmation=0,
                param1=1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0
            )
            rospy.loginfo(f"EKF重置命令执行：iris_{uav.uavID}, success={response.success}")
            rospy.sleep(1.0)
            # 重启EKF估计器
            response = cmdService(
                broadcast=False,
                command=214,  # MAV_CMD_DO_SET_PARAMETER
                confirmation=0,
                param1=1, param2=1, param3=0, param4=0, param5=0, param6=0, param7=0
            )
            rospy.loginfo(f"EKF重启命令执行：iris_{uav.uavID}, success={response.success}")
        except Exception as e:
            rospy.logwarn(f"重新校准命令失败：iris_{uav.uavID}, {e}")

    def _provide_initial_state(self, uav: UAV) -> None:
        """
        主动向EKF提供新的初始位姿，帮助收敛
        :param uav: 无人机对象
        :return: None
        """
        # 位姿发布
        visionPosePub = rospy.Publisher(f"/iris_{uav.uavID}/mavros/vision_pose/pose", PoseStamped, queue_size=10)
        # 速度发布
        velocityPub = rospy.Publisher(f"/iris_{uav.uavID}/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        # 点位信息
        poseMsg = PoseStamped()
        poseMsg.header.frame_id = "map"
        poseMsg.pose = self.make_pose(uav.initPosition.x, uav.initPosition.y, uav.initPosition.z, np.pi / 2)
        # 速度信息
        velocityMsg = TwistStamped()
        velocityMsg.header.frame_id = "base_link"
        velocityMsg.twist.linear.x, velocityMsg.twist.linear.y, velocityMsg.twist.linear.z = 0.0, 0.0, 0.0
        velocityMsg.twist.angular.x, velocityMsg.twist.angular.y, velocityMsg.twist.angular.z = 0.0, 0.0, 0.0
        # 发布
        rate = rospy.Rate(20)
        for _ in range(20):
            poseMsg.header.stamp = rospy.Time.now()
            velocityMsg.header.stamp = rospy.Time.now()
            visionPosePub.publish(poseMsg)
            velocityPub.publish(velocityMsg)
            rate.sleep()
        rospy.loginfo(f"已向PX4提供初始状态：iris_{uav.uavID}")

    def _publish_maker(self) -> None:
        """
        消息发布
        :return: None
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