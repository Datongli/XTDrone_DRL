"""
目标类
"""
class Target:
    def __init__(self, uavID: int, x: int| float, y: int| float, z: int| float) -> None:
        """目标基本信息"""
        self.uavID: int = uavID  # 目标点对应的无人机ID
        self.x: int| float = x  # 目标点x坐标
        self.y: int| float = y  # 目标点y坐标
        self.z: int| float = z  # 目标点z坐标



if __name__ == "__main__":
    pass