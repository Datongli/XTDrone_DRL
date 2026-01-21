"""
该文件用于ego-planner环境
目前只支持单无人机
"""
from env.scripts.env import StaticObstacleEnv
import rospy
from geometry_msgs.msg import PoseStamped
import subprocess
import os
import traceback
import subprocess
from std_msgs.msg import Empty
import numpy as np
import time
import threading
from rospy.msg import AnyMsg
from std_msgs.msg import Empty, String


class EGOPlannerEnv(StaticObstacleEnv):
    """
    适用于ego-planner的避障环境
    继承自StaticObstacleEnv
    """

    def __init__(self, cfg, mode: str="test"):
        """
        构造函数
        :param cfg: 环境配置
        :return: None
        """
        super().__init__(cfg, mode=mode)
        try:
            self.uavs[0].communication.enable_target_motion_pub = False
            rospy.logwarn("[ego] 已关闭本进程 target_motion -> mavros raw setpoint 发布（由ego-planner接管）")
        except Exception as e:
            rospy.logwarn(f"[ego] 关闭 target_motion 发布失败: {e}")
        self._init_ego_goal_publishers()
        self._init_ego_planner()

    def reset(self) -> tuple:
        """
        重置环境，并添加和ego-planner相关的配置
        :return: 无人机信息元组
        """
        # 获取重置后的环境
        uavInformation =  super().reset(seed=getattr(self.cfg, "seed", None))
        """gazebo物理仿真继续运行"""
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except rospy.ServiceException as e:
            rospy.logerr("取消暂停gazebo物理仿真失败: %s", e)
        """与ego-planner相关"""
        # 禁掉本节点对MAVROS raw setpoint的发布
        self._disable_env_mavros_raw_setpoint_publishers(uavID=0)
        self._disable_env_xtdrone_cmd_publishers(uavID=0)
        # 启动ego-planner规划节点
        self._start_ego_planner_nodes()
        # 发布目标点
        self._publish_ego_goals()
        # 切换回OFFBOARD
        self._ensure_offboard(uavID=0)
        return uavInformation
    
    def step(self) -> tuple | None:
        """
        步进
        """
        """gazebo物理仿真继续运行"""
        rospy.wait_for_service("/gazebo/unpause_physics")
        # try:
        #     self.unpause()
        # except rospy.ServiceException as e:
        #     rospy.logerr("取消暂停gazebo物理仿真失败: %s", e)
        """一定的时间步长"""
        rate = rospy.Rate(50)
        for _ in range(50):
            rate.sleep()
        """gazebo物理仿真暂停"""
        # try:
        #     self.pause()
        # except rospy.ServiceException as e:
        #     rospy.logerr("暂停gazebo物理仿真失败: %s", e)

    def _generate_multi_uav_launch(self, launchFile: str) -> None:
        """
        子类重写
        生成无人机launch文件
        :param launchFile: launch文件路径
        :return: None
        """
        try:
            # launch所在文件夹
            launchDir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "launch")
            # 模版文件路径
            templateFile = os.path.join(launchDir, "ego_planner_uav.launch.template")
        except Exception as e:
            rospy.logerr(f"打开launch文件失败: {e}")
            traceback.print_exc()
            return
        try:
            # 修改launch文件
            with open(launchFile, 'w') as f:
                with open(templateFile, 'r') as template:
                    content = template.read()
                    content = content.format(
                        # uavX=self.length / 2.0,
                        uavX=80.0,
                        uavY=5.0,
                        uavZ=0.5,
                        uavYaw=np.pi / 2
                    )
                    f.write(content)
            rospy.loginfo(f"成功生成launch文件: {launchFile}")
        except Exception as e:
            rospy.logerr(f"生成launch文件失败: {e}")
            traceback.print_exc()

    def _init_ego_goal_publishers(self) -> None:
        """
        初始化给 ego-planner 发布目标点的 publishers
        """
        # uavNums 在父类里初始化；这里兜底一下
        uavNums = int(getattr(self, "uavNums", 1))
        self.egoGoalPublishers = []  # 发布目标点的 publishers
        for i in range(uavNums):
            # global话题
            globalTopic = f"/move_base_simple/goal_{i}" if uavNums > 1 else "/move_base_simple/goal"
            # 命名空间话题
            nsTopic = f"/iris_{i}" + (f"/move_base_simple/goal_{i}" if uavNums > 1 else "/move_base_simple/goal")
            # xtdrone命名空间
            xtdroneNsTopic = f"/xtdrone/iris_{i}" + (f"/move_base_simple/goal_{i}" if uavNums > 1 else "/move_base_simple/goal")
            pubs = {
                "global": rospy.Publisher(globalTopic, PoseStamped, queue_size=1, latch=True),
                "ns": rospy.Publisher(nsTopic, PoseStamped, queue_size=1, latch=True),
                "xtdrone_ns": rospy.Publisher(xtdroneNsTopic, PoseStamped, queue_size=1, latch=True),
            }
            self.egoGoalPublishers.append(pubs)

    
    def _init_ego_planner(self) -> None:
        """
        初始化ego-planner所需的基础服务
        按照官方教程启动VINS-Fusion、话题转换、通信和坐标系转换
        :return: None
        """
        rospy.loginfo("开始初始化ego-planner基础服务...")
        # 存储所有启动的进程，便于结束的时候清理
        self.egoProcesses: list = []
        try:
            # 启动VINS-Fusion
            self._start_vins_fusion()
            rospy.sleep(0.5)  # 等待服务稳定
            # 为无人机启动支持服务
            rospy.loginfo(f"为无人机启动ego_planner支持服务...")
            # 启动VINS话题转换
            self._start_vins_transfer()
            rospy.sleep(0.5)  # 等待服务稳定
            # 启动通信脚本
            self._start_communication()
            rospy.sleep(0.5)  # 等待服务稳定
            # 启动相机坐标转换脚本
            self._start_ego_transfer()
            rospy.sleep(0.5)  # 等待服务稳定
            rospy.loginfo("ego-planner基础服务初始化完成")
        except Exception as e:
            rospy.logerr(f"初始化ego-planner基础服务失败: {e}")
            # 清理已启动的进程
            self._cleanup_ego_processes()
            raise

    def _start_vins_fusion(self) -> None:
        """
        启动VINS-Fusion视觉里程计
        :return: None
        """
        rospy.loginfo("启动VINS-Fusion视觉里程计...")
        # VINS-Fusion的工作目录
        catkinWorkSpace = os.path.expanduser("~/catkin_ws")
        vinsScript = os.path.join(catkinWorkSpace, "scripts/xtdrone_run_vio.sh")
        # 检查脚本是否存在
        if not os.path.exists(vinsScript):
            rospy.logwarn(f"VINS脚本不存在:{vinsScript}，尝试直接启动launch文件")
            # 备用方案，直接用launch文件启动
            try:
                vinsLaunch = "roslaunch vins vins_rviz.launch"
                process = subprocess.Popen(
                    vinsLaunch,
                    shell=True,
                    cwd=catkinWorkSpace,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception as e:
                rospy.logerr(f"从launch文件启动VINS-Fusion失败: {e}")
                raise
        else:
            try:
                # 脚本存在使用脚本启动
                process = subprocess.Popen(
                    ['bash', vinsScript],
                    cwd=catkinWorkSpace,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception as e:
                rospy.logerr(f"从脚本启动VINS-Fusion失败: {e}")
                raise
        # 记录进程
        self.egoProcesses.append(process)
        rospy.loginfo("VINS-Fusion启动成功")
        rospy.sleep(0.5)  # 短暂延时等待VINS初始化

    def _start_vins_transfer(self) -> None:
        """
        启动VINS话题转换脚本
        将Odometry类型转换为PX4所需的话题
        :return: None
        """
        rospy.loginfo(f"启动VINS话题转换...")
        # VINS话题转换脚本路径
        vinsTransferPath = os.path.expanduser("~/XTDrone/sensing/slam/vio/vins_transfer.py")
        # 若脚本不存在则返回
        if not os.path.exists(vinsTransferPath):
            rospy.logerr(f"VINS转换脚本不存在：{vinsTransferPath}")
            return
        # 启动转换脚本
        try:
            process = subprocess.Popen(
                ['python', vinsTransferPath, 'iris', '0'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            # 记录进程
            self.egoProcesses.append(process)
            rospy.loginfo(f"VINS话题转换启动成功!")
        except Exception as e:
            rospy.logerr(f"从脚本启动VINS话题转换失败: {e}")

    def _start_communication(self) -> None:
        """
        启动通信脚本
        :return: None
        """
        rospy.loginfo(f"启动通信脚本...")
        # 通信脚本路径
        communicationPath = os.path.expanduser("~/XTDrone/communication/multirotor_communication.py")
        # 若脚本不存在则返回
        if not os.path.exists(communicationPath):
            rospy.logerr(f"通信脚本不存在：{communicationPath}")
            return
        # 启动通信脚本
        try:
            process = subprocess.Popen(
                ['python', communicationPath, 'iris', '0'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            # 记录进程
            self.egoProcesses.append(process)
            rospy.loginfo(f"通信脚本启动成功!")
        except Exception as e:
            rospy.logerr(f"从脚本启动通信脚本失败: {e}")   

    def _start_ego_transfer(self) -> None:
        """
        启动ego坐标系转换脚本
        转换相机位姿坐标系方向
        :return: None
        """
        rospy.loginfo(f"启动相机位姿坐标系转换...")
        # 相机坐标转换脚本路径
        egoTransferPath = os.path.expanduser("~/XTDrone/motion_planning/3d/ego_transfer.py")
        # 若脚本不存在则返回
        if not os.path.exists(egoTransferPath):
            rospy.logerr(f"相机坐标转换脚本不存在：{egoTransferPath}")
            return
        # 启动转换脚本
        try:
            process = subprocess.Popen(
                ['python', egoTransferPath, 'iris', '0'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            # 记录进程
            self.egoProcesses.append(process)
            rospy.loginfo(f"相机位姿坐标系转换启动成功!")
        except Exception as e:
            rospy.logerr(f"从脚本启动相机位姿坐标系转换失败: {e}")   

    def _cleanup_ego_processes(self) -> None:
        """
        清理ego-planner相关进程
        :return: None
        """
        rospy.loginfo("清理ego-planner相关进程...")
        for process in getattr(self, "egoProcesses", []):
            try:
                process.terminate()
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
            except Exception as e:
                rospy.logerr(f"清理ego-planner进程失败：{e}")
        self.egoProcesses = []
        rospy.loginfo("ego-planner进程清理完毕")

    def _start_ego_planner_nodes(self) -> None:
        """
        启动ego-planner规划节点
        此时目标点已经生成，可以发布给ego-planner
        :return: None
        """
        if not hasattr(self, "egoPlannerStarted"):
            rospy.loginfo("启动ego-planner规划节点...")
            # 生成单个无人机 ego-planner 的launch文件
            launchFile = self._generate_ego_planner_launch()
            if not launchFile:
                rospy.logerr("ego-planner launch 文件生成失败，无法启动")
                return
            # 把 ego-planner 输出写入日志文件
            logPath = f"/home/ldt/文档/ego_planner_log/ego_planner_iris_0_{int(time.time())}.log"
            rospy.loginfo(f"正在启动ego-planner，日志输出到: {logPath}")
            # 启动ego-planner
            try:
                rospy.loginfo(f"正在启动ego-planner")
                logFile = open(logPath, "w")
                process = subprocess.Popen(
                    ['roslaunch', launchFile],
                    stdout=logFile,
                    stderr=logFile,
                )
                # 等待并检查进程状态
                rospy.sleep(0.5)
                if process.poll() is not None:
                    rospy.logerr(f"ego-planner 进程退出 (返回码: {process.returncode})")
                    rospy.logerr(f"请查看日志: {logPath}")
                else:
                    rospy.loginfo(f"ego-planner 运行中 (PID: {process.pid})")
                    self.egoProcesses.append(process)
            except Exception as e:
                rospy.logerr(f"ego-planner 启动失败: {e}")
                rospy.logerr(traceback.format_exc())  
            self.egoPlannerStarted = True  # 标志位置位
            rospy.loginfo("ego-planner规划节点启动完成")
            rospy.sleep(0.5)  # 等待ego-planner初始化

    def _generate_ego_planner_launch(self) -> str| None:
        """
        生成单个无人机 ego-planner 的launch文件
        :return: launch文件路径
        """
        try:
            # launch所在文件夹
            launchDir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "launch")
            os.makedirs(launchDir, exist_ok=True)  # 创建launch所在文件夹
            # launch文件路径
            launchFile = os.path.join(launchDir, f"single_uav.launch")
            # 模版文件路径
            templateFile = os.path.join(launchDir, "single_uav.launch.template")
            # 检查模板文件是否存在
            if not os.path.exists(templateFile):
                rospy.logerr(f"模板文件不存在：{templateFile}")
                return None
            # 使用真实map_origin
            mapOriginX, mapOriginY = self._get_map_origin(uavID=0)
            # 目标点统一使用 world 绝对坐标（与 grid_map/frame_id=world 对齐）
            goalX = float(self.targets[0].x)
            goalY = float(self.targets[0].y)
            goalZ = float(self.targets[0].z)

            with open(launchFile, 'w') as f:
                with open(templateFile, 'r') as template:
                    content = template.read()
                    content = content.format(
                        mapSizeX=self.cfg.env.length * 5,
                        mapSizeY=self.cfg.env.width * 5,
                        mapSizeZ=self.cfg.env.height * 5,
                        mapOriginX=0.0,
                        mapOriginY=0.0,
                        targetX=goalX,
                        targetY=goalY,
                        targetZ=goalZ
                    )
                    f.write(content)
            rospy.loginfo(f"成功生成ego-planner launch文件: {launchFile}")
            # 验证生成的文件
            if not os.path.exists(launchFile):
                rospy.logerr(f"Launch文件生成失败: {launchFile}")
                return None
            return launchFile
        except Exception as e:
            rospy.logerr(f"生成ego-planner launch文件失败: {e}")
            return None
        
    def _get_map_origin(self, uavID: int = 0) -> tuple[float, float]:
        """从 MAVROS 读取当前本地坐标原点，避免与 initPosition 不一致。"""
        poseTopic = f"/iris_{uavID}/mavros/local_position/pose"
        try:
            msg = rospy.wait_for_message(poseTopic, PoseStamped, timeout=2.0)
            return float(msg.pose.position.x), float(msg.pose.position.y)
        except Exception:
            return float(self.cfg.uav.initPosition.x), float(self.cfg.uav.initPosition.y)
        
    def _verify_ego_planner_nodes(self) -> None:
        """
        验证ego-planner节点是否启动成功
        :return: None
        """
        rospy.loginfo("=" * 60)
        rospy.loginfo("验证 ego-planner 节点状态...")
        result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True)
        nodes = result.stdout.split('\n')
        egoNodes = [n for n in nodes if ('ego' in n.lower() or 'planner' in n.lower()) and n.strip()]
        if egoNodes:
            rospy.loginfo("发现ego-planner相关节点：")
            for node in egoNodes:
                rospy.loginfo(f"  - {node}")
        else:
            rospy.logerr("没有发现任何ego-planner节点")
        # 检查目标点话题
        result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True)
        topics = result.stdout.split('\n')
        rospy.loginfo("\n检查目标点话题...")
        for i in range(self.uavNums):
            goalTopic = f"/move_base_simple/goal_{i}" if self.uavNums > 1 else "/move_base_simple/goal"
            if goalTopic in topics:
                rospy.loginfo(f"{goalTopic} 存在")
            else:
                rospy.logerr(f"{goalTopic} 不存在")
        # 检查规划话题
        rospy.loginfo("\n检查规划输出话题:")
        for i in range(self.uavNums):
            bsplineTopic = f"/iris_{i}/planning/bspline"
            if bsplineTopic in topics:
                rospy.loginfo(f"{bsplineTopic}")
            else:
                rospy.logwarn(f"{bsplineTopic} (规划未开始,等待目标点)")
        rospy.loginfo("=" * 60)

    def _publish_ego_goals(self) -> None:
        """
        发布目标点给ego-planner
        在reset中调用，此时目标点已生成
        :return: None
        """
        """发布目标点给ego-planner"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("发布目标点给ego-planner...")
        self._debug_setpoint_sources()
        # 等深度链路/占据更新相关订阅者挂上
        self._wait_depth_pipeline(timeout=5.0)
        # 等待订阅者出现（最多 3s）
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < 3.0:
            ok = True
            for i in range(self.uavNums):
                pubs = self.egoGoalPublishers[i]
                if (pubs["global"].get_num_connections() == 0 
                    and pubs["ns"].get_num_connections() == 0
                    and pubs["xtdrone_ns"].get_num_connections() == 0):
                    ok = False
                    break
            if ok:
                break
            rospy.sleep(0.1)
        # 直接发布目标点
        rospy.loginfo("发布目标点...")
        mapOriginX, mapOriginY = self._get_map_origin(uavID=0)
        for i in range(self.uavNums):
            goalMsg = PoseStamped()
            goalMsg.header.frame_id = "world"  # 与 grid_map/frame_id 对齐
            goalMsg.header.stamp = rospy.Time.now()
            goal_x = float(self.targets[i].x)
            goal_y = float(self.targets[i].y)
            goal_z = float(self.targets[i].z)
            goalMsg.pose.position.x = goal_x
            goalMsg.pose.position.y = goal_y
            goalMsg.pose.position.z = goal_z
            goalMsg.pose.orientation.x = 0.0
            goalMsg.pose.orientation.y = 0.0
            goalMsg.pose.orientation.z = 0.0
            goalMsg.pose.orientation.w = 1.0
            rospy.loginfo(
                f"iris_{i} 目标(map): ({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f})"
            )
            pubs = self.egoGoalPublishers[i]
            # 优先发给有订阅者的一侧；如果两侧都没订阅者，就两边都发（兜底）
            ns_conn = pubs["ns"].get_num_connections()
            gl_conn = pubs["global"].get_num_connections()
            xtdrone_ns_conn = pubs["xtdrone_ns"].get_num_connections()
            for _ in range(30):
                if ns_conn > 0:
                    pubs["ns"].publish(goalMsg)
                if gl_conn > 0:
                    pubs["global"].publish(goalMsg)
                if xtdrone_ns_conn > 0:
                    pubs["xtdrone_ns"].publish(goalMsg)
                if ns_conn == 0 and gl_conn == 0 and xtdrone_ns_conn == 0:
                    pubs["ns"].publish(goalMsg)
                    pubs["global"].publish(goalMsg)
                    pubs["xtdrone_ns"].publish(goalMsg)
                rospy.sleep(0.05)
            rospy.sleep(0.5)
        rospy.loginfo("目标点发布完成")
        rospy.loginfo("=" * 60)
        # 补发trigger
        self._publish_trigger_if_needed()
        rospy.sleep(1.0)
        self._verify_planning_started()

    def _debug_setpoint_sources(self) -> None:
        """打印 MAVROS setpoint 相关话题的发布者，判断控制权是否被抢走。"""
        topics = [
            "/iris_0/mavros/setpoint_position/local",
            "/iris_0/mavros/setpoint_trajectory/desired",
            "/iris_0/mavros/setpoint_raw/local",
        ]
        for topic in topics:
            try:
                info = subprocess.run(["rostopic", "info", topic], capture_output=True, text=True)
                rospy.logwarn(f"[DEBUG] rostopic info {topic}:\n{info.stdout}")
            except Exception as e:
                rospy.logwarn(f"[DEBUG] 查询 {topic} 失败: {e}")

    def _wait_depth_pipeline(self, timeout: float = 5.0) -> None:
        """
        等待深度点云/深度图至少有一帧数据，避免 ego-planner 立刻 Depth Lost/空地图
        """
        depthPoints = "/iris_0/realsense/depth_camera/depth/points"
        depthImage = "/iris_0/realsense/depth_camera/depth/image_raw"
        cameraPose = "/iris_0/camera_pose"
        t0 = time.time()
        # 先等订阅者
        while time.time() - t0 < timeout:
            try:
                info = subprocess.run(["rostopic", "info", depthPoints], capture_output=True, text=True)
                if "Subscribers" in info.stdout and ("*" in info.stdout or "No subscribers" not in info.stdout):
                    break
            except Exception:
                pass
            rospy.sleep(0.2)
        # 再等一帧数据（AnyMsg 不依赖具体消息类型）
        try:
            rospy.wait_for_message(depthPoints, AnyMsg, timeout=timeout)
        except Exception:
            try:
                rospy.wait_for_message(depthImage, AnyMsg, timeout=timeout)
            except Exception:
                pass
        # 额外等一帧相机位姿，减少坐标系未就绪导致的“在障碍里”
        try:
            rospy.wait_for_message(cameraPose, PoseStamped, timeout=timeout)
        except Exception:
            pass

    def _publish_trigger_if_needed(self) -> None:
        """
        ego-planner 日志显示会等待 trigger（来自 n3ctrl/RC）。
        这里自动向常见 trigger 话题发布 Empty，提升确定性。
        """
        triggerCandidates = [
            "/traj_start_trigger",
            "/iris_0/traj_start_trigger",
            "/xtdrone/iris_0/traj_start_trigger",
            "/planning/start_trigger",
            "/iris_0/planning/start_trigger",
            "/xtdrone/iris_0/planning/start_trigger",
        ]
        # 取一帧当前位姿作为 trigger 内容
        poseTopic = "/iris_0/mavros/local_position/pose"
        try:
            curPose = rospy.wait_for_message(poseTopic, PoseStamped, timeout=1.0)
        except Exception as e:
            rospy.logwarn(f"[trigger] 等待当前位姿失败({poseTopic}): {e}")
            curPose = PoseStamped()
            curPose.header.frame_id = "map"
            curPose.header.stamp = rospy.Time.now()
            curPose.pose.orientation.w = 1.0
        curPose.header.stamp = rospy.Time.now()
        pubbed = False
        for t in triggerCandidates:
            try:
                pub = rospy.Publisher(t, PoseStamped, queue_size=1, latch=True)
                for _ in range(5):
                    pub.publish(curPose)
                    rospy.sleep(0.05)
                rospy.loginfo(f"[trigger] 已发布 PoseStamped 到: {t}")
                pubbed = True
            except Exception:
                continue
        if not pubbed:
            rospy.logwarn("[trigger] 未找到可用 trigger 话题/或发布失败")

    def _verify_planning_started(self) -> None:
        """
        验证规划是否开始
        :return: None
        """
        rospy.loginfo("验证规划是否已开始...")
        result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True)
        topics = result.stdout.split('\n')
        bsplineTopics = self._find_bspline_topics(uav_id=0)
        if bsplineTopics:
            rospy.loginfo(f"iris_0 发现 bspline 话题: {bsplineTopics}")
            rospy.loginfo("iris_0 规划可能已开始（至少已有 bspline 输出话题）")
        else:
            rospy.logwarn("iris_0 未发现任何包含 'bspline' 的话题（可能未开始规划，或命名空间不同）")
            # 额外打印一些线索，方便你对齐话题名
            topics = self._list_topics()
            hints = [t for t in topics if t.startswith("/iris_0/") and any(k in t.lower() for k in ["planning", "traj", "path", "odom", "depth"])]
            rospy.logwarn(f"iris_0 相关话题线索(部分): {hints[:50]}")

    def _find_bspline_topics(self, uav_id: int = 0) -> list[str]:
        """自动查找可能的 bspline 话题（避免硬编码写错）"""
        topics = self._list_topics()
        prefixes = [f"/iris_{uav_id}/", f"/xtdrone/iris_{uav_id}/"]
        cands = []
        for t in topics:
            tl = t.lower()
            if "bspline" not in tl:
                continue
            if any(t.startswith(p) for p in prefixes) or t == "/broadcast_bspline":
                cands.append(t)
        return sorted(set(cands))
    
    def _list_topics(self) -> list[str]:
        """返回当前 rosmaster 上的 topic 列表"""
        try:
            return [t for t, _ in rospy.get_published_topics()]
        except Exception:
            result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True)
            return [t for t in result.stdout.splitlines() if t.strip()]
        
    def _ensure_offboard(self, uavID: int = 0) -> None:
        """让实际控制 raw setpoint 的通信节点(/iris_0_communication)切回OFFBOARD。"""
        try:
            topic = f"/xtdrone/iris_{uavID}/cmd"
            pub = rospy.Publisher(topic, String, queue_size=1, latch=True)
            msg = String(data="OFFBOARD")
            for _ in range(10):
                pub.publish(msg)
                rospy.sleep(0.05)
            rospy.loginfo(f"iris_{uavID}: 已通过 {topic} 发布 OFFBOARD")
        except Exception as e:
            rospy.logwarn(f"iris_{uavID}: 发布 OFFBOARD 失败: {e}")

    def _disable_env_mavros_raw_setpoint_publishers(self, uavID: int = 0) -> None:
        """
        关键修复：禁止本进程(/static_obstacle_env_*)继续发布 /iris_x/mavros/setpoint_raw/local
        否则会覆盖 /iris_x_communication 的输出，导致“直飞不避障”。
        """
        targetTopic = f"/iris_{uavID}/mavros/setpoint_raw/local"

        def _maybe_unregister(obj) -> int:
            n = 0
            for name, val in list(vars(obj).items()):
                # rospy.Publisher 类型判断用 duck-typing 更稳
                if not hasattr(val, "unregister"):
                    continue
                # Publisher 一般有 resolved_name/name 字段
                pubName = getattr(val, "resolved_name", None) or getattr(val, "name", None)
                if pubName == targetTopic:
                    try:
                        val.unregister()
                        setattr(obj, name, None)
                        n += 1
                    except Exception as e:
                        rospy.logwarn(f"[disable_raw_sp] 注销Publisher失败 {name}@{pubName}: {e}")
            return n

        total = 0
        # 在这些对象里扫：env自身、uav对象、uav.communication（最常见发布点）
        try:
            total += _maybe_unregister(self)
        except Exception:
            pass
        try:
            if hasattr(self, "uavs") and len(self.uavs) > uavID:
                total += _maybe_unregister(self.uavs[uavID])
                if hasattr(self.uavs[uavID], "communication"):
                    total += _maybe_unregister(self.uavs[uavID].communication)
        except Exception:
            pass
        rospy.logwarn(f"[disable_raw_sp] 已对 {targetTopic} 注销 {total} 个Publisher（来自 /static_obstacle_env_* 进程）")

    def _disable_env_xtdrone_cmd_publishers(self, uavID: int = 0) -> None:
        """
        禁止本进程(/static_obstacle_env_*)继续发布 xtdrone 的 cmd_* 话题，
        否则会覆盖 /iris_0_traj_server 发来的避障指令。
        """
        topics = {
            f"/xtdrone/iris_{uavID}/cmd_pose_enu",
            f"/xtdrone/iris_{uavID}/cmd_pose_flu",
            f"/xtdrone/iris_{uavID}/cmd_vel_enu",
            f"/xtdrone/iris_{uavID}/cmd_vel_flu",
            f"/xtdrone/iris_{uavID}/cmd_accel_enu",
            f"/xtdrone/iris_{uavID}/cmd_accel_flu",
        }

        def _maybe_unregister(obj) -> int:
            n = 0
            for name, val in list(vars(obj).items()):
                if not hasattr(val, "unregister"):
                    continue
                pubName = getattr(val, "resolved_name", None) or getattr(val, "name", None)
                if pubName in topics:
                    try:
                        val.unregister()
                        setattr(obj, name, None)
                        n += 1
                    except Exception as e:
                        rospy.logwarn(f"[disable_xtdrone_cmd] 注销Publisher失败 {name}@{pubName}: {e}")
            return n

        total = 0
        for obj in [self, *getattr(self, "uavs", [])]:
            try:
                total += _maybe_unregister(obj)
                comm = getattr(obj, "communication", None)
                if comm is not None:
                    total += _maybe_unregister(comm)
            except Exception:
                pass
        rospy.logwarn(f"[disable_xtdrone_cmd] 已注销 {total} 个 xtdrone cmd_* Publisher（来自 /static_obstacle_env_* 进程）")
    
    def __del__(self) -> None:
        """
        析构函数
        """
        # 清理ego-planner相关进程
        if hasattr(self, 'egoProcesses'):
            self._cleanup_ego_processes()




if __name__ == "__main__":
    pass