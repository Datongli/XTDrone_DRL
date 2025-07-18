"""
障碍物类
"""
from dataclasses import dataclass


@dataclass
class Obstacle:
    """障碍物类"""
    x: int| float = 0  # 障碍物x坐标
    y: int| float = 0  # 障碍物y坐标
    z: int| float = 0  # 障碍物z坐标
    halfLength: int| float = 0  # 障碍物半长
    halfWidth: int| float = 0  # 障碍物半宽
    height: int| float = 0  # 障碍物高度


if __name__ == "__main__":
    pass