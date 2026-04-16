import math
from array import array

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker


def quaternion_from_yaw(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def yaw_from_north_cw_deg(heading_deg: float) -> float:
    return math.radians(90.0 - heading_deg)


class VirtualRobotFromCmdVel(Node):
    def __init__(self) -> None:
        super().__init__("virtual_robot_from_cmdvel")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("frames.path_frame", "map"),
                ("frames.odom_frame", "odom"),
                ("frames.base_frame", "base_link"),
                ("topics.cmd_vel", "/cmd_vel"),
                ("topics.robot_marker", "/move_pkg/sim_robot_marker"),
                ("topics.path", "/move_pkg/sim_path"),
                ("topics.odom", "/move_pkg/odom"),
                ("topics.scan", "/scan"),
                ("sim.update_hz", 20.0),
                ("sim.max_path_points", 2000),
                ("sim.initial_heading_deg_from_north_cw", 0.0),
                ("sim.publish_fake_scan", True),
                ("sim.fake_scan_hz", 10.0),
                ("sim.fake_scan_range_max", 30.0),
            ],
        )
        self.path_frame = str(self.get_parameter("frames.path_frame").value)
        self.odom_frame = str(self.get_parameter("frames.odom_frame").value)
        self.base_frame = str(self.get_parameter("frames.base_frame").value)
        self.max_path_points = int(self.get_parameter("sim.max_path_points").value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = yaw_from_north_cw_deg(
            float(self.get_parameter("sim.initial_heading_deg_from_north_cw").value)
        )
        self.vx = 0.0
        self.wz = 0.0

        cmd_vel_topic = str(self.get_parameter("topics.cmd_vel").value)
        robot_marker_topic = str(self.get_parameter("topics.robot_marker").value)
        path_topic = str(self.get_parameter("topics.path").value)
        odom_topic = str(self.get_parameter("topics.odom").value)
        scan_topic = str(self.get_parameter("topics.scan").value)

        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 20)
        self.marker_pub = self.create_publisher(Marker, robot_marker_topic, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 20)
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.path_frame

        hz = float(self.get_parameter("sim.update_hz").value)
        self.dt = 1.0 / max(1e-6, hz)
        self.create_timer(self.dt, self._on_timer)
        self.publish_fake_scan = bool(self.get_parameter("sim.publish_fake_scan").value)
        self.fake_scan_range_max = float(self.get_parameter("sim.fake_scan_range_max").value)
        if self.publish_fake_scan:
            fake_scan_hz = float(self.get_parameter("sim.fake_scan_hz").value)
            self.create_timer(1.0 / max(1e-6, fake_scan_hz), self._publish_fake_scan)
        self.get_logger().info(f"가상 로봇 노드 시작: cmd_vel={cmd_vel_topic}")

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.vx = float(msg.linear.x)
        self.wz = float(msg.angular.z)

    def _on_timer(self) -> None:
        self.x += self.vx * math.cos(self.yaw) * self.dt
        self.y += self.vx * math.sin(self.yaw) * self.dt
        self.yaw += self.wz * self.dt

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)

        marker = Marker()
        marker.header.frame_id = self.path_frame
        marker.header.stamp = stamp
        marker.ns = "sim_robot"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.15
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.3
        marker.color.r = 0.9
        marker.color.g = 0.5
        marker.color.b = 0.1
        marker.color.a = 0.95
        self.marker_pub.publish(marker)

        pose = marker.pose
        path_pose = self._build_path_pose(stamp, pose)
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(path_pose)
        if len(self.path_msg.poses) > self.max_path_points:
            self.path_msg.poses = self.path_msg.poses[-self.max_path_points :]
        self.path_pub.publish(self.path_msg)
        self._publish_tf_and_odom(stamp, qx, qy, qz, qw)

    def _build_path_pose(self, stamp, pose):  # noqa: ANN001
        from geometry_msgs.msg import PoseStamped

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.path_frame
        pose_stamped.header.stamp = stamp
        pose_stamped.pose = pose
        return pose_stamped

    def _publish_tf_and_odom(self, stamp, qx: float, qy: float, qz: float, qw: float) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

    def _publish_fake_scan(self) -> None:
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.base_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)
        sample_count = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = self.fake_scan_range_max
        scan.ranges = array("f", [scan.range_max] * sample_count)
        scan.intensities = array("f", [0.0] * sample_count)
        self.scan_pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VirtualRobotFromCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
