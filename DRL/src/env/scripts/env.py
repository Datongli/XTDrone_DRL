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
from geometry_msgs.msg import Pose, Point, Quaternion
from env.scripts.target import Target
from env.scripts.obstacle import Obstacle
from visualization_msgs.msg import MarkerArray, Marker
from gazebo_msgs.msg import ModelState
from uav.scripts.uav import UAV


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
        self.uavs: list[UAV] = [UAV(i) for i in range(self.uavNums)]
        """设置topic发布者、订阅者、服务"""
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)          # 恢复物理模拟
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)              # 暂停物理模拟
        self.resetProxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)           # 重置世界服务
        self.deleteModel = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)   # 删除模型服务
        self.spawnModel = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)  # 生成模型服务
        self.targetVisualization = rospy.Publisher("/target", MarkerArray, queue_size=3)  # 目标点可视化
        self.uavSetState  = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=10)  # 无人机位置和姿态初始化设置
        
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
        rospy.loginfo("环境重置完成")
        

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

    def _generate_multi_uav_launch(self, launchFile: str):
        """
        生成多无人机launch文件
        """
        """构造多无人机launch文件"""
        # launch文件头
        header = '''<launch>
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
                <arg name="sdf" value="iris_realsense_camera"/>
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
                    y=5,
                    z=0.5,
                    R=0.0,
                    P=0.0,
                    Y=math.pi/2,
                    mavlinkUdpPort=18570+i,
                    mavlinkTcpPort=4560+i,
                ))
            f.write(footer)

    def _wait_for_gazebo_services(self):
        """等待 Gazebo 服务启动"""
        services_to_wait = [
            "/gazebo/unpause_physics",
            "/gazebo/pause_physics", 
            "/gazebo/reset_world",
            "/gazebo/delete_model",
            "/gazebo/spawn_sdf_model"
        ]

        for service_name in services_to_wait:
            rospy.loginfo(f"等待服务: {service_name}")
            try:
                rospy.wait_for_service(service_name, timeout=30)  # 30秒超时
                rospy.loginfo(f"服务 {service_name} 已就绪")
            except rospy.ROSException:
                rospy.logerr(f"等待服务 {service_name} 超时!")
                raise

        rospy.loginfo("所有 Gazebo 服务已就绪")
        # 等待 MAVROS 节点启动
        self._wait_for_mavros_nodes()

    def _wait_for_mavros_nodes(self):
        """等待所有 MAVROS 节点启动"""
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

    def _uav_reset(self) -> None:
        """
        所有无人机归位并悬停（并行批量处理）
        """
        """归位所有无人机"""
        stateMsgs = []
        uav_target_positions = []  # 记录每个无人机的目标位置
        rate = rospy.Rate(50)  # 50Hz
        
        for uav in self.uavs:
            stateMsg = ModelState()
            stateMsg.model_name = "iris_" + str(uav.uavID)
            # 计算初始位置
            initial_x = self.length // self.uavNums * uav.uavID + self.length / (2 * self.uavNums)
            initial_y = 5
            initial_z = 0.5
            
            stateMsg.pose.position.x = initial_x
            stateMsg.pose.position.y = initial_y
            stateMsg.pose.position.z = initial_z
            
            # 计算目标悬停位置（在初始位置上方5米）
            target_x = 0
            target_y = 0
            target_z = 5.0
            uav_target_positions.append((target_x, target_y, target_z))
            
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
                pose = Pose()
                # 使用计算好的绝对目标位置
                pose.position.x = uav_target_positions[j][0]
                pose.position.y = uav_target_positions[j][1] 
                pose.position.z = uav_target_positions[j][2]
                
                angle = np.pi / 2
                quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
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
        
        """启动并悬停无人机 - 使用绝对目标位置"""
        rospy.loginfo("开始发布目标位置...")
        
        
        # 延长发布时间，确保PX4接收到足够的setpoint
        for i in range(500):  # 发布10秒
            for j, uav in enumerate(self.uavs):
                pose = Pose()
                # 使用计算好的绝对目标位置
                pose.position.x = uav_target_positions[j][0]
                pose.position.y = uav_target_positions[j][1] 
                pose.position.z = uav_target_positions[j][2]
                
                angle = np.pi / 2
                quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
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
        for i in range(300):  # 继续发布15秒
            for j, uav in enumerate(self.uavs):
                pose = Pose()
                # 继续使用绝对目标位置
                pose.position.x = uav_target_positions[j][0]
                pose.position.y = uav_target_positions[j][1]
                pose.position.z = uav_target_positions[j][2]
                
                angle = np.pi / 2
                quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
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
            
            # rospy.loginfo(f"已起飞无人机数量: {readyNum}/{self.uavNums}")
            
            if readyNum == self.uavNums:
                rospy.loginfo("所有无人机已到达目标高度")
                break
                
            # 继续发布setpoint
            for j, uav in enumerate(self.uavs):
                pose = Pose()
                pose.position.x = uav_target_positions[j][0]
                pose.position.y = uav_target_positions[j][1]
                pose.position.z = uav_target_positions[j][2]
                
                angle = np.pi / 2
                quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
                uav.shotTargetPub.publish(pose)
            
            rate.sleep()
            timeout_count += 1
        
        if timeout_count >= max_timeout:
            rospy.logwarn("部分无人机未能到达目标高度，继续执行...")
        
        # 切换到悬停状态
        rospy.loginfo("切换到悬停状态...")
        for uav in self.uavs:
            cmdMsg = String()
            cmdMsg.data = "HOVER"
            uav.communication.cmd_callback(cmdMsg)
        
        # 等待无人机稳定
        rospy.sleep(2)

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