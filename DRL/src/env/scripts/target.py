"""
目标类
"""
from dataclasses import dataclass


@dataclass
class Target:
    """目标类"""
    x: int| float = 0  # 目标点x坐标
    y: int| float = 0  # 目标点y坐标
    z: int| float = 0  # 目标点z坐标


if __name__ == "__main__":
    pass