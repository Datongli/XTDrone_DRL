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


class EGOPlannerEnv(StaticObstacleEnv):
    """
    适用于ego-planner的避障环境
    继承自StaticObstacleEnv
    """

    def __init__(self, cfg):
        """
        构造函数
        :param cfg: 环境配置
        :return: None
        """
        super().__init__(cfg)
        self._init_ego_planner()

    def reset(self) -> tuple:
        """
        重置环境，并添加和ego-planner相关的配置
        :return: 无人机信息元组
        """
        # 获取重置后的环境
        uavInformation =  super().reset()
        """gazebo物理仿真继续运行"""
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except rospy.ServiceException as e:
            rospy.logerr("取消暂停gazebo物理仿真失败: %s", e)
        """与ego-planner相关"""
        # 启动ego-planner规划节点
        self._start_ego_planner_nodes()
        # 发布目标点
        # self._publish_ego_goals()
        """gazebo物理仿真暂停"""
        try:
            self.pause()
        except rospy.ServiceException as e:
            rospy.logerr("暂停gazebo物理仿真失败: %s", e)
        return uavInformation
    
    def step(self) -> tuple | None:
        """
        步进
        """
        """gazebo物理仿真继续运行"""
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except rospy.ServiceException as e:
            rospy.logerr("取消暂停gazebo物理仿真失败: %s", e)
        """一定的时间步长"""
        rate = rospy.Rate(50)
        for _ in range(50):
            rate.sleep()
        """gazebo物理仿真暂停"""
        try:
            self.pause()
        except rospy.ServiceException as e:
            rospy.logerr("暂停gazebo物理仿真失败: %s", e)

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
                        uavX=self.length / 2.0,
                        uavY=5.0,
                        uavZ=0.5,
                        uavYaw=np.pi / 2
                    )
                    f.write(content)
            rospy.loginfo(f"成功生成launch文件: {launchFile}")
        except Exception as e:
            rospy.logerr(f"生成launch文件失败: {e}")
            traceback.print_exc()
    
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
                vinsLaunch = "launch vins vins_rviz.launch"
                process = subprocess.Popen(
                    vinsLaunch,
                    shell=True,
                    cwd=catkinWorkSpace,
                    stdout=subprocess.PIPE
                )
            except Exception as e:
                rospy.logerr(f"从launch文件启动VINS-Fusion失败: {e}")
        else:
            try:
                # 脚本存在使用脚本启动
                process = subprocess.Popen(
                    ['bash', vinsScript],
                    cwd=catkinWorkSpace,
                    stdout=subprocess.PIPE
                )
            except Exception as e:
                rospy.logerr(f"从脚本启动VINS-Fusion失败: {e}")
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
                stdout=subprocess.PIPE,
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
                stdout=subprocess.PIPE,
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
                stdout=subprocess.PIPE,
                # stderr=subprocess.PIPE
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
            # 启动ego-planner
            try:
                rospy.loginfo(f"正在启动ego-planner")
                process = subprocess.Popen(
                    ['roslaunch', launchFile],
                    stdout=subprocess.PIPE,
                )
                # 等待并检查进程状态
                rospy.sleep(0.5)
                if process.poll() is not None:
                    rospy.logerr(f"ego-planner 进程退出 (返回码: {process.returncode})")
                    rospy.logerr(f"请检查launch文件: {launchFile}")
                else:
                    rospy.loginfo(f"ego-planner 运行中 (PID: {process.pid})")
                    self.egoProcesses.append(process)
            except Exception as e:
                rospy.logerr(f"ego-planner 启动失败: {e}")
                rospy.logerr(traceback.format_exc())  
            self.egoPlannerStarted = True  # 标志位置位
            rospy.loginfo("ego-planner规划节点启动完成")
            rospy.sleep(0.5)  # 等待ego-planner初始化
            # self._verify_ego_planner_nodes()

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
            # 打开并修改launch文件
            with open(launchFile, 'w') as f:
                with open(templateFile, 'r') as template:
                    content = template.read()
                    content = content.format(
                        mapSizeX=self.cfg.env.length,
                        mapSizeY=self.cfg.env.width,
                        mapSizeZ=self.cfg.env.height,
                        targetX=self.targets[0].x - self.length / 2.0,
                        targetY=self.targets[0].y - 4.5,
                        targetZ=self.targets[0].z
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
        # 减少等待时间 - 订阅者会消失!
        rospy.sleep(3.0)  # 从 10.0 减少到 3.0
        # 直接发布目标点
        rospy.loginfo("发布目标点...")
        for i in range(self.uavNums):
            goalMsg = PoseStamped()
            goalMsg.header.frame_id = "world"
            goalMsg.header.stamp = rospy.Time.now()
            goalMsg.pose.position.x = self.targets[i].x
            goalMsg.pose.position.y = self.targets[i].y
            goalMsg.pose.position.z = self.targets[i].z
            goalMsg.pose.orientation.x = 0.0
            goalMsg.pose.orientation.y = 0.0
            goalMsg.pose.orientation.z = 0.0
            goalMsg.pose.orientation.w = 1.0
            rospy.loginfo(f"iris_{i} 目标: ({goalMsg.pose.position.x:.2f}, "
                        f"{goalMsg.pose.position.y:.2f}, {goalMsg.pose.position.z:.2f})")
            # 持续发布以确保接收
            for _ in range(50):
                self.egoGoalPublishers[i].publish(goalMsg)
                rospy.sleep(0.05)
            rospy.sleep(1.0)
        rospy.loginfo("目标点发布完成")
        rospy.loginfo("=" * 60)
        rospy.sleep(3.0)
        self._verify_planning_started()

    def _verify_planning_started(self) -> None:
        """
        验证规划是否开始
        :return: None
        """
        rospy.loginfo("验证规划是否已开始...")
        result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True)
        topics = result.stdout.split('\n')
        for i in range(self.uavNums):
            bsplineTopic = f"/iris_{i}/planning/bspline"
            if bsplineTopic in topics:
                rospy.loginfo(f"iris_{i} 规划已开始 (bspline话题存在)")
            else:
                rospy.logwarn(f"iris_{i} 规划可能未开始 (bspline话题不存在)")

    def __del__(self) -> None:
        """
        析构函数
        """
        # 清理ego-planner相关进程
        if hasattr(self, 'egoProcesses'):
            self._cleanup_ego_processes()




if __name__ == "__main__":
    pass