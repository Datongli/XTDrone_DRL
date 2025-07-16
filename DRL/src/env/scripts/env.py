import gymnasium as gym
import rospy


class StaticObstacleEnv(gym.Env):
    def __init__(self):
        pass    

    def step(self):
        pass

    def reset(self):
        """
        重置环境
        重新随机生成障碍物，将无人机重置到初始位置并悬停，规划无人机目标点
        发布目标点和无人机状态，传感器数据
        """
        pass


if __name__ == "__main__":
    pass