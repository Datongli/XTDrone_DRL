# 介绍
XTDrone-DRL是基于[XTDrone](https://github.com/robin-shaun/XTDrone)框架的深度强化学习无人机路径规划算法仿真平台，目前仅支持四旋翼无人机的仿真。
主要设计目的是在XTDrone框架的基础上，编写深度强化学习的训练环境，实现四旋翼无人机在训练环境中起飞、悬停、飞行、碰撞/到达目标点/超过步长、重置环境等强化学习训练过程。
建议使用者具备一定的ROS1，Gazebo，Rviz基础。
使用者应按照[XTDrone框架使用教程](https://www.yuque.com/xtdrone/manual_cn)创建自己的工作空间，再将该项目的源代码视为多个ROS包放入编译。

训练过程采用课程学习的思想，课程难度等级意味着环境中障碍物数量的多少。为了提升算法的收敛速度，提取数学抽象特征，基于Gymnasium库，搭建了[简易的DRL训练环境](https://github.com/Datongli/Drone-self-avoidance/tree/master/self_avoidance_gym_refactor)，将算法可以先置于Gymnasium的环境下先行训练，然后再在Gazebo中训练，极大减小算法收敛的训练时间。
[完整版算法演示视频](https://youtu.be/Wi3fjBdrXjQ)

# 项目结构
- other
  - XTDrone框架自带的文件，包括各个ROS节点的构建，传感器的仿真，飞控的配置等
- DRL
  - 作者的主要代码
  - src
    - env
      - 深度强化学习训练环境的源代码
      - launch
        - 环境配套的启动文件
        - 环境的world模版，可以更改环境的长宽高
      - models
        - 环境中的地面和障碍物贴图建模
      - rviz
        - Rviz中的配置文件，默认启动后加载到Rviz中
      - scripts
        - 环境的源代码
        - 障碍物、目标点的源代码
        - 轨迹可视化节点的源代码
    - navigation
      - 路径规划算法源代码
      - scripts
        - MTrans-SACs算法源代码
        - DDPG算法源代码等
    - test
      - 用于测试算法性能的文件
      - scripts
        - 工具文件
        - 测试文件源代码
    - train
      - 训练源代码
      - cfg
        - 配置文件，用于配置环境、无人机传感器等
      - scripts
        - 训练源代码
        - 工具源代码
    - uav
      - 无人机源文件
      - scripts
        - 传感器文件
        - 无人机文件

# 教程
留待后续完善

# 作者
北京航空航天大学 电子信息工程学院 李大同
联系作者：Ldt20020517@buaa.edu.cn

如果想与XTDrone团队建立合作，请联系卓安<zhuoan@stu.pku.edu.cn>。
