#include"../../task_main/task_main.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower2_main"); //初始化ROS节点，节点名
    TASK_MAIN follower2_main;
    if (true)
    {
        follower2_main.set_planeID(2);
        follower2_main.run();
    }
    return 0;
}
