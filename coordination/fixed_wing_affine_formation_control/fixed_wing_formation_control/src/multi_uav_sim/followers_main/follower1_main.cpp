#include"../../task_main/task_main.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower1_main"); //初始化ROS节点，节点名
    TASK_MAIN follower1_main;
    if (true)
    {
        follower1_main.set_planeID(1);
        follower1_main.run();
    }
    return 0;
}
