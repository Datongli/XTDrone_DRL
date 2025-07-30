import rospy
from mavros_msgs.msg import PositionTarget, ParamValue, State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import String
from pyquaternion import Quaternion

import sys



class Communication:

    def __init__(self, vehicle_type, vehicle_id):
        """状态变量初始化"""
        self.vehicle_type = vehicle_type  # 无人机类型
        self.vehicle_id = vehicle_id  # 无人机ID
        self.current_position = None  # 相对于home点的位置（本地位置）
        self.current_yaw = 0  # 当前偏航角
        self.hover_flag = 0  # 悬停标志
        self.coordinate_frame = 1  # 坐标框架（1=ENU， 9=FLU）
        self.target_motion = PositionTarget()  # 目标运动
        self.target_motion.coordinate_frame = self.coordinate_frame  # 目标运动坐标框架
        self.arm_state = False  # 解锁状态
        self.motion_type = 0  # 运动类型
        self.flight_mode = None  # 飞行模式
        self.mission = None  # 任务
        self.last_cmd = None
        self.rate = rospy.Rate(30)
        # 等待MAVROS连接状态
        mavros_state = rospy.wait_for_message(self.vehicle_type+'_'+self.vehicle_id+"/mavros/state", State)
        if not mavros_state.connected:
            rospy.logwarn(self.vehicle_type+'_'+self.vehicle_id+": No connection to FCU. Check mavros!")
            exit(0)

        ####################
        ## ROS 接口 ##
        ####################
        """ROS 订阅者"""
        # 本地位姿
        self.local_pose_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback,queue_size=1)
        # 监听无人机指令
        self.cmd_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd",String,self.cmd_callback,queue_size=3)
        # 监听无人机位置指令(flu和enu坐标系下)
        self.cmd_pose_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_flu", Pose, self.cmd_pose_flu_callback,queue_size=1)
        self.cmd_pose_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_enu", Pose, self.cmd_pose_enu_callback,queue_size=1)
        # 监听无人机速度指令(flu和enu坐标系下)
        self.cmd_vel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_flu", Twist, self.cmd_vel_flu_callback,queue_size=1)
        self.cmd_vel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_enu", Twist, self.cmd_vel_enu_callback,queue_size=1)
        # 监听无人机加速度指令(flu和enu坐标系下)
        self.cmd_accel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_accel_flu", Twist, self.cmd_accel_flu_callback,queue_size=1)
        self.cmd_accel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_accel_enu", Twist, self.cmd_accel_enu_callback,queue_size=1)
        ''' 
        ros publishers
        '''
        # 目标运动发布者
        self.target_motion_pub = rospy.Publisher(self.vehicle_type+'_'+self.vehicle_id+"/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        '''
        ros services
        '''
        # 解锁服务
        self.armService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/cmd/arming", CommandBool)
        # 切换飞行模式服务
        self.flightModeService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/set_mode", SetMode)
        # 设置参数服务
        self.set_param_srv = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/param/set", ParamSet)
        rcl_except = ParamValue(4, 0.0)
        self.set_param_srv("COM_RCL_EXCEPT", rcl_except)

        print(self.vehicle_type+'_'+self.vehicle_id+": "+"communication initialized")

    def start(self):
        '''
        main ROS thread
        '''
        while not rospy.is_shutdown():
            self.target_motion.header.stamp = rospy.Time(0)
            self.target_motion_pub.publish(self.target_motion)
            self.rate.sleep()

    def local_pose_callback(self, msg):
        """本地位姿回调函数"""
        self.current_position = msg.pose.position  # 更新当前位置
        self.current_yaw = self.q2yaw(msg.pose.orientation)  # 更新当前偏航角

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        """构造目标运动"""
        target_raw_pose = PositionTarget()

        target_raw_pose.header.stamp = rospy.Time(0)

        target_raw_pose.coordinate_frame = self.coordinate_frame
        
        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz
        
        target_raw_pose.acceleration_or_force.x = afx
        target_raw_pose.acceleration_or_force.y = afy
        target_raw_pose.acceleration_or_force.z = afz

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        if(self.motion_type == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW_RATE
        if(self.motion_type == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW
        if(self.motion_type == 2):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_YAW

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame = 9
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=yaw)
 
    def cmd_pose_enu_callback(self, msg):
        """位置指令回调函数"""
        self.coordinate_frame = 1  # 位置指令坐标系
        self.motion_type = 0  # 位置控制模式
        yaw = self.q2yaw(msg.orientation)  # 将四元数转换为偏航角
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=yaw)
        
    def cmd_vel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)  
 
    def cmd_vel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)    

    def cmd_accel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 2
            self.target_motion = self.construct_target(ax=msg.linear.x,ay=msg.linear.y,az=msg.linear.z,yaw_rate=msg.angular.z)    
            
    def cmd_accel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1 
            self.motion_type = 2
            self.target_motion = self.construct_target(ax=msg.linear.x,ay=msg.linear.y,az=msg.linear.z,yaw_rate=msg.angular.z)    
            
    def hover_state_transition(self,x,y,z,w):
        if abs(x) > 0.02 or abs(y)  > 0.02 or abs(z)  > 0.02 or abs(w)  > 0.005:
            self.hover_flag = 0
            self.flight_mode = 'OFFBOARD'
        elif not self.flight_mode == "HOVER":
            self.hover_flag = 1
            self.flight_mode = 'HOVER'
            self.hover()
    
    def cmd_callback(self, msg: String):
        """无人机指令回调函数"""
        rospy.loginfo(f"{self.vehicle_type}_{self.vehicle_id}: 收到指令 {msg.data}")
        # 过滤
        if msg.data == self.last_cmd or msg.data == '' or msg.data == 'stop controlling':
            return
        # ARM指令
        elif msg.data == 'ARM':
            # 调用self.arm()方法尝试解锁无人机
            self.arm_state =self.arm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))
        # DISARM指令
        elif msg.data == 'DISARM':
            # 调用self.disarm()方法尝试上锁无人机
            self.arm_state = not self.disarm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))
        # 以mission开头且与当前任务不同的指令
        elif msg.data[:-1] == "mission" and not msg.data == self.mission:
            self.mission = msg.data
            print(self.vehicle_type+'_'+self.vehicle_id+": "+msg.data)
        # 其他指令  
        else:
            # 状态切换
            self.flight_mode = msg.data
            self.flight_mode_switch()

        self.last_cmd = msg.data

    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad
    
    def arm(self):
        """解锁无人机"""
        # 调用armService服务，参数为True表示解锁
        if self.armService(True):
            return True
        # 解锁失败输出提示
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": arming failed!")
            return False

    def disarm(self):
        """上锁无人机"""
        # 调用armService服务，参数为False表示上锁
        if self.armService(False):
            return True
        # 上锁失败输出提示
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": disarming failed!")
            return False

    def hover(self):
        self.coordinate_frame = 1
        self.motion_type = 0
        self.target_motion = self.construct_target(x=self.current_position.x,y=self.current_position.y,z=self.current_position.z,yaw=self.current_yaw)
        print(self.vehicle_type+'_'+self.vehicle_id+":"+self.flight_mode)

    def flight_mode_switch(self):
        """切换无人机飞行模式"""
        # 悬停模式
        if self.flight_mode == 'HOVER':
            self.hover_flag = 1
            self.hover()
        # 其他模式
        elif self.flightModeService(custom_mode=self.flight_mode):
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode)
            return True
        # 切换失败
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode+"failed")
            return False

if __name__ == '__main__':
    rospy.init_node(sys.argv[1]+'_'+sys.argv[2]+"_communication")
    communication = Communication(sys.argv[1],sys.argv[2])
    communication.start()
