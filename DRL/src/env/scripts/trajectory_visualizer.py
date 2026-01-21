#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""轨迹可视化节点（RViz）
功能：
1) 订阅无人机位姿（geometry_msgs/PoseStamped）
2) 按距离阈值采样并累计轨迹点
3) 同时发布两种 RViz 友好的可视化消息：
    - nav_msgs/Path：适合 RViz 的 Path 显示
    - visualization_msgs/Marker(LINE_STRIP)：适合论文截图（可控线宽/颜色）
4) 提供 reset 服务：一键清空轨迹

注意：
- 本节点不会创建 TF；frame_id 必须是 TF 中存在的坐标系（例如 map/odom）。
- 为了保证 RViz 后启动也能看到轨迹，本节点对发布者启用了 latch。
"""

import math
from typing import List, Optional
import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker


class TrajectoryVisualizer:
    def __init__(self) -> None:
        """初始化节点：读取参数、创建发布/订阅、准备缓存。"""
        # ===== 1) ROS 参数（保持参数名不变，兼容 launch 文件） =====
        # 轨迹发布使用的坐标系名称（必须在 TF 树中存在，否则 RViz 会报 Frame 不存在）
        self.frameId: str = rospy.get_param("~frame_id", "map")
        # 订阅的位姿话题（无人机实际位姿源，一般来自 mavros）
        self.poseTopic: str = rospy.get_param("~pose_topic", "/iris_0/mavros/vision_pose/pose")
        # 输出话题：Path 与 Marker，两者都能在 RViz 中显示轨迹
        self.pathTopic: str = rospy.get_param("~path_topic", "/iris_0/trajectory/path")
        self.markerTopic: str = rospy.get_param("~marker_topic", "/iris_0/trajectory/marker")
        # ===== 2) 轨迹采样与显示参数 =====
        # 两个相邻轨迹点的最小距离（米）。小则轨迹更平滑但点更多；大则更稀疏、性能更好。
        self.minDist: float = float(rospy.get_param("~min_dist", 0.05))
        # 最大保留点数，防止长时间运行导致内存/渲染开销过大
        self.maxPoints: int = int(rospy.get_param("~max_points", 50000))
        # 线宽（Marker 的 scale.x），单位米；截图时可调得更细
        self.lineWidth: float = float(rospy.get_param("~line_width", 0.1))
        # 颜色（RGBA），例如 [1,0,0,1] 为不透明红色
        colorRgba = rospy.get_param("~color_rgba", [1.0, 0.1, 0.1, 1.0])
        if not isinstance(colorRgba, list) or len(colorRgba) != 4:
            colorRgba = [1.0, 0.1, 0.1, 1.0]
        self.colorRgba = [float(c) for c in colorRgba]
        # ===== 3) 运行时缓存 =====
        # 轨迹点缓存（每个点是 geometry_msgs/Point）
        self.points: List[Point] = []
        # 上一个被采样/记录的点，用于 minDist 判定
        self.lastPoint: Optional[Point] = None
        # ===== 4) ROS 发布者（latch=True：RViz 后启动也能看到最后一次发布的消息） =====
        self.pathPub = rospy.Publisher(self.pathTopic, Path, queue_size=1, latch=True)
        self.markerPub = rospy.Publisher(self.markerTopic, Marker, queue_size=1, latch=True)
        # ===== 5) ROS 服务与订阅者 =====
        # 私有服务：~reset（清空已记录轨迹）
        self.resetSrv = rospy.Service("~reset", Empty, self._handle_reset)
        # 订阅位姿：收到新位姿时更新轨迹
        self.poseSub = rospy.Subscriber(self.poseTopic, PoseStamped, self._pose_cb, queue_size=200)
        # 打印关键配置，便于排查 RViz 不显示/Frame 不存在等问题
        rospy.loginfo(f"[trajectory_visualizer] pose_topic={self.poseTopic}")
        rospy.loginfo(f"[trajectory_visualizer] path_topic={self.pathTopic}")
        rospy.loginfo(f"[trajectory_visualizer] marker_topic={self.markerTopic}")
        rospy.loginfo(f"[trajectory_visualizer] frame_id={self.frameId}, min_dist={self.minDist}m")

    def _handle_reset(self, _req) -> EmptyResponse:
        """处理 ~reset 服务请求：清空轨迹缓存，并发布“删除/空消息”让 RViz 立即刷新。"""
        # 1) 清空缓存
        self.points.clear()
        self.lastPoint = None
        # 清空 Marker（RViz 里会立即消失）
        # 2) 清空 Marker（DELETEALL 会删除该发布者相关的所有 Marker）
        marker = Marker()
        marker.header.frame_id = self.frameId
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory"
        marker.id = 0
        marker.action = Marker.DELETEALL
        self.markerPub.publish(marker)
        # 清空 Path
        # 3) 发布空 Path
        path = Path()
        path.header.frame_id = self.frameId
        path.header.stamp = rospy.Time.now()
        self.pathPub.publish(path)
        return EmptyResponse()

    def _pose_cb(self, msg: PoseStamped) -> None:
        """位姿回调：按 minDist 采样，更新并发布 Path/Marker。"""
        # 1) 从 PoseStamped 中提取当前位置
        p = msg.pose.position
        cur = Point(x=float(p.x), y=float(p.y), z=float(p.z))
        # 2) 与上一个记录点距离过近则跳过，降低发布频率/点数
        if self.lastPoint is not None:
            dx = cur.x - self.lastPoint.x
            dy = cur.y - self.lastPoint.y
            dz = cur.z - self.lastPoint.z
            if math.sqrt(dx * dx + dy * dy + dz * dz) < self.minDist:
                return
        # 3) 记录新点
        self.points.append(cur)
        self.lastPoint = cur
        # 4) 控制点数上限（防止 RViz 越跑越卡）
        if len(self.points) > self.maxPoints:
            drop = len(self.points) - self.maxPoints
            self.points = self.points[drop:]
        now = rospy.Time.now()
        # 发布 Path（RViz 的 Path 显示很方便）
        # 5) 发布 Path：由 PoseStamped 序列组成
        path = Path()
        path.header.frame_id = self.frameId
        path.header.stamp = now
        poses: List[PoseStamped] = []
        for pt in self.points:
            ps = PoseStamped()
            ps.header.frame_id = self.frameId
            ps.header.stamp = now
            ps.pose.position = pt
            ps.pose.orientation.w = 1.0
            poses.append(ps)
        path.poses = poses
        self.pathPub.publish(path)
        # 发布 LineStrip Marker（论文截图一般更“细线条”更好看）
        # 6) 发布 Marker：LINE_STRIP 直接连线（论文截图通常更直观）
        marker = Marker()
        marker.header.frame_id = self.frameId
        marker.header.stamp = now
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = self.lineWidth
        marker.color.r = self.colorRgba[0]
        marker.color.g = self.colorRgba[1]
        marker.color.b = self.colorRgba[2]
        marker.color.a = self.colorRgba[3]
        marker.pose.orientation.w = 1.0
        marker.points = list(self.points)
        marker.lifetime = rospy.Duration(0.0)
        self.markerPub.publish(marker)


def main() -> None:
    """主入口：初始化 ROS 节点并进入 spin 循环。"""
    rospy.init_node("trajectory_visualizer", anonymous=True)
    TrajectoryVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
