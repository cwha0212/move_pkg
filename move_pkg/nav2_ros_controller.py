import json
import math
import threading
import time
from typing import Tuple

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


EARTH_RADIUS_M = 6378137.0


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
) -> Tuple[float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    origin_lat = math.radians(origin_lat_deg)
    origin_lon = math.radians(origin_lon_deg)

    d_lat = lat - origin_lat
    d_lon = lon - origin_lon
    east = d_lon * math.cos(origin_lat) * EARTH_RADIUS_M
    north = d_lat * EARTH_RADIUS_M
    return east, north


def yaw_from_north_cw_deg(heading_deg: float) -> float:
    return math.radians(90.0 - heading_deg)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def format_float_full_precision(value: float) -> str:
    return format(float(value), ".17g")


class Nav2RosController(Node):
    def __init__(self) -> None:
        super().__init__("nav2_ros_controller")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("start_pose.lat", 37.4115),
                ("start_pose.lon", 127.0931),
                ("start_pose.heading_deg_from_north_cw", 0.0),
                ("ligo.use_current_as_start_on_mission", True),
                ("ligo.anchor_on_goal", True),
                ("ligo.reanchor_each_goal", True),
                ("ligo.max_data_age_sec", 2.0),
                ("ligo.sync_tolerance_sec", 0.2),
                ("topics.move_status", "/move_status"),
                ("topics.goal_latlon_sub", "/move_pkg/nav/goal"),
                ("topics.goal_received_pub", "/move_pkg/nav/goal_received"),
                ("topics.reached_pub", "/move_pkg/nav/reached"),
                ("topics.ligo_global_position_sub", "/ligo/global_position"),
                ("topics.ligo_heading_deg_sub", "/ligo/enu_heading_deg"),
                ("topics.current_heading_pub", "/move_pkg/nav/current_heading"),
                ("topics.current_enu_pub", "/move_pkg/nav/current_enu"),
                ("frames.path_frame", "map"),
                ("frames.mission_enu_frame", "mission_enu"),
                ("frames.base_frame", "base_link"),
                ("nav.server_wait_timeout_sec", 15.0),
            ],
        )

        self.path_frame = str(self.get_parameter("frames.path_frame").value)
        self.origin_marker_topic = "/move_pkg/enu_origin_marker"
        self.goal_latlon_sub_topic = str(
            self.get_parameter("topics.goal_latlon_sub").value
        )
        self.goal_received_pub_topic = str(
            self.get_parameter("topics.goal_received_pub").value
        )
        self.reached_pub_topic = str(self.get_parameter("topics.reached_pub").value)
        self.ligo_global_position_sub_topic = str(
            self.get_parameter("topics.ligo_global_position_sub").value
        )
        self.ligo_heading_deg_sub_topic = str(
            self.get_parameter("topics.ligo_heading_deg_sub").value
        )
        self.current_heading_pub_topic = str(
            self.get_parameter("topics.current_heading_pub").value
        )
        self.current_enu_pub_topic = str(
            self.get_parameter("topics.current_enu_pub").value
        )

        self.origin_lat = float(self.get_parameter("start_pose.lat").value)
        self.origin_lon = float(self.get_parameter("start_pose.lon").value)
        self.heading_deg = float(
            self.get_parameter("start_pose.heading_deg_from_north_cw").value
        )
        self.mission_enu_frame = str(self.get_parameter("frames.mission_enu_frame").value)

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._goal_request_seq = 0
        self._map_meta = None
        self._latest_ligo_lat = None
        self._latest_ligo_lon = None
        self._latest_ligo_heading_deg = None
        self._latest_ligo_position_stamp_sec = None
        self._latest_ligo_heading_stamp_sec = None
        self._mission_anchor_lat = None
        self._mission_anchor_lon = None
        self._mission_anchor_heading_deg = None
        self._anchor_history_points = []
        self._anchor_history_reference_lat = None
        self._anchor_history_reference_lon = None

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.origin_marker_pub = self.create_publisher(
            Marker, self.origin_marker_topic, 10
        )
        move_status_topic = str(self.get_parameter("topics.move_status").value)
        self.move_status_pub = self.create_publisher(Bool, move_status_topic, 10)
        self.goal_received_pub = self.create_publisher(
            String, self.goal_received_pub_topic, 10
        )
        self.reached_pub = self.create_publisher(String, self.reached_pub_topic, 10)
        self.current_heading_pub = self.create_publisher(
            Float64, self.current_heading_pub_topic, 10
        )
        self.current_enu_pub = self.create_publisher(
            PointStamped, self.current_enu_pub_topic, 10
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self.create_subscription(
            NavSatFix, self.goal_latlon_sub_topic, self._on_goal_navsat, 10
        )
        self.create_subscription(
            NavSatFix, self.ligo_global_position_sub_topic, self._on_ligo_global_position, 10
        )
        self.create_subscription(
            Float64,
            self.ligo_heading_deg_sub_topic,
            self._on_ligo_heading_deg,
            10,
        )

        self._publish_origin_marker()

        self.get_logger().info(
            "Nav2 controller 시작: origin=("
            f"{format_float_full_precision(self.origin_lat)}, "
            f"{format_float_full_precision(self.origin_lon)}), "
            f"heading={self.heading_deg:.2f} deg"
        )
        self.get_logger().info(
            f"목표 ROS 토픽 구독: {self.goal_latlon_sub_topic} (sensor_msgs/NavSatFix)"
        )
        self.get_logger().info(
            f"목표 수신 ACK 토픽 발행: {self.goal_received_pub_topic} (std_msgs/String)"
        )
        self.get_logger().info(
            f"LIGO 위치 토픽 구독: {self.ligo_global_position_sub_topic} (sensor_msgs/NavSatFix)"
        )
        self.get_logger().info(
            f"LIGO heading 토픽 구독: {self.ligo_heading_deg_sub_topic} (std_msgs/Float64, deg from north cw)"
        )
        self.get_logger().info(
            f"실시간 heading 토픽 발행: {self.current_heading_pub_topic} (std_msgs/Float64)"
        )
        self.get_logger().info(
            f"anchor 기준 현재 ENU 토픽 발행: {self.current_enu_pub_topic} (geometry_msgs/PointStamped)"
        )

    def _on_goal_navsat(self, msg: NavSatFix) -> None:
        goal_lat = float(msg.latitude)
        goal_lon = float(msg.longitude)
        if not math.isfinite(goal_lat) or not math.isfinite(goal_lon):
            self.get_logger().error("ROS goal 수신값 오류: latitude/longitude가 유한값이 아닙니다.")
            self._publish_goal_received_ack(
                goal_lat=goal_lat,
                goal_lon=goal_lon,
                command_failed=True,
                reason="목표 좌표가 유효하지 않습니다. latitude/longitude 값을 확인해주세요.",
            )
            self._publish_move_status(False)
            return
        if not self._resolve_mission_anchor():
            self._publish_goal_received_ack(
                goal_lat=goal_lat,
                goal_lon=goal_lon,
                command_failed=True,
                reason="현재 위치를 확인할 수 없어 목표를 처리할 수 없습니다. LIGO 위치 수신 후 다시 시도해주세요.",
            )
            self._publish_move_status(False)
            return
        self._publish_goal_received_ack(
            goal_lat=goal_lat,
            goal_lon=goal_lon,
            command_failed=False,
            reason="목표 명령이 정상 접수되었습니다.",
        )
        self._handle_goal(goal_lat=goal_lat, goal_lon=goal_lon)

    def _on_ligo_global_position(self, msg: NavSatFix) -> None:
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if not math.isfinite(lat) or not math.isfinite(lon):
            return
        self._latest_ligo_lat = lat
        self._latest_ligo_lon = lon
        self._latest_ligo_position_stamp_sec = self.get_clock().now().nanoseconds / 1e9
        self._publish_current_enu_from_anchor()

    def _on_ligo_heading_deg(self, msg: Float64) -> None:
        heading_deg = float(msg.data) % 360.0
        if not math.isfinite(heading_deg):
            return
        self._latest_ligo_heading_deg = heading_deg
        self._latest_ligo_heading_stamp_sec = self.get_clock().now().nanoseconds / 1e9
        self._publish_current_heading(self._latest_ligo_heading_deg)

    def _update_start_pose_from_ligo_if_enabled(self) -> bool:
        if not bool(self.get_parameter("ligo.use_current_as_start_on_mission").value):
            return True
        if self._latest_ligo_lat is None or self._latest_ligo_lon is None:
            self.get_logger().error(
                "LIGO 현재 위치 미수신: 목표를 처리할 수 없습니다. LIGO 위치 수신 후 다시 시도해주세요."
            )
            return False
        now_sec = self.get_clock().now().nanoseconds / 1e9
        max_age_sec = float(self.get_parameter("ligo.max_data_age_sec").value)
        if self._latest_ligo_position_stamp_sec is None or (
            now_sec - self._latest_ligo_position_stamp_sec > max_age_sec
        ):
            self.get_logger().error(
                "LIGO 위치 데이터가 오래되었습니다: 목표를 처리할 수 없습니다. 최신 위치 수신 후 다시 시도해주세요."
            )
            return False
        heading_deg = self.heading_deg
        if self._latest_ligo_heading_deg is not None and self._latest_ligo_heading_stamp_sec is not None:
            heading_is_fresh = now_sec - self._latest_ligo_heading_stamp_sec <= max_age_sec
            sync_tolerance_sec = float(self.get_parameter("ligo.sync_tolerance_sec").value)
            sync_delta_sec = abs(
                self._latest_ligo_position_stamp_sec - self._latest_ligo_heading_stamp_sec
            )
            if heading_is_fresh and sync_delta_sec <= sync_tolerance_sec:
                heading_deg = float(self._latest_ligo_heading_deg)
            else:
                self.get_logger().warn(
                    "LIGO 위치/heading 동기화 불충분: "
                    f"delta={sync_delta_sec:.3f}s (tol={sync_tolerance_sec:.3f}s). "
                    "heading은 기존 값을 사용합니다."
                )
        self.origin_lat = float(self._latest_ligo_lat)
        self.origin_lon = float(self._latest_ligo_lon)
        self.heading_deg = float(heading_deg)
        self._publish_origin_marker()
        self.get_logger().info(
            "mission start를 LIGO 현재 위치로 갱신: "
            f"lat={format_float_full_precision(self.origin_lat)}, "
            f"lon={format_float_full_precision(self.origin_lon)}, "
            f"heading={self.heading_deg:.2f}"
        )
        return True

    def _resolve_mission_anchor(self) -> bool:
        anchor_on_goal = bool(self.get_parameter("ligo.anchor_on_goal").value)
        reanchor_each_goal = bool(self.get_parameter("ligo.reanchor_each_goal").value)
        if not anchor_on_goal:
            if not self._update_start_pose_from_ligo_if_enabled():
                return False
            self._mission_anchor_lat = self.origin_lat
            self._mission_anchor_lon = self.origin_lon
            self._mission_anchor_heading_deg = self.heading_deg
            self._reset_mission_enu_origin()
            self._record_anchor_history(self._mission_anchor_lat, self._mission_anchor_lon)
            return True

        need_new_anchor = (
            self._mission_anchor_lat is None
            or self._mission_anchor_lon is None
            or self._mission_anchor_heading_deg is None
            or reanchor_each_goal
        )
        if need_new_anchor:
            if not self._update_start_pose_from_ligo_if_enabled():
                return False
            self._mission_anchor_lat = self.origin_lat
            self._mission_anchor_lon = self.origin_lon
            self._mission_anchor_heading_deg = self.heading_deg
            self._reset_mission_enu_origin()
            self._record_anchor_history(self._mission_anchor_lat, self._mission_anchor_lon)
            self.get_logger().info(
                "mission anchor 확정: "
                f"lat={format_float_full_precision(self._mission_anchor_lat)}, "
                f"lon={format_float_full_precision(self._mission_anchor_lon)}, "
                f"heading={self._mission_anchor_heading_deg:.2f}"
            )
        else:
            self.origin_lat = float(self._mission_anchor_lat)
            self.origin_lon = float(self._mission_anchor_lon)
            self.heading_deg = float(self._mission_anchor_heading_deg)
        return True

    def _handle_goal(self, goal_lat: float, goal_lon: float) -> None:
        if not self.nav_client.wait_for_server(
            timeout_sec=float(self.get_parameter("nav.server_wait_timeout_sec").value)
        ):
            self.get_logger().error("Nav2 액션 서버 대기 시간 초과")
            self._publish_move_status(False)
            return

        east, north = geodetic_to_enu(
            goal_lat, goal_lon, self.origin_lat, self.origin_lon
        )
        goal_bearing_yaw = math.atan2(north, east)
        planned_goal_yaw = goal_bearing_yaw
        if self._map_meta is not None:
            mx0 = self._map_meta["origin_x"]
            my0 = self._map_meta["origin_y"]
            mx1 = mx0 + self._map_meta["width"] * self._map_meta["resolution"]
            my1 = my0 + self._map_meta["height"] * self._map_meta["resolution"]
        yaw = planned_goal_yaw
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.path_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = east
        goal_msg.pose.pose.position.y = north
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        with self._goal_lock:
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    pass

        self._goal_request_seq += 1
        goal_request_id = self._goal_request_seq
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda fut: self._on_goal_response(
                fut, goal_lat=goal_lat, goal_lon=goal_lon, goal_request_id=goal_request_id
            )
        )
        self.get_logger().info(
            "Nav2 goal 전송: "
            f"lat={format_float_full_precision(goal_lat)} "
            f"lon={format_float_full_precision(goal_lon)} "
            f"-> ENU x={east:.2f} y={north:.2f}"
        )

    def _on_goal_response(
        self, future, goal_lat: float, goal_lon: float, goal_request_id: int
    ) -> None:  # noqa: ANN001
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"goal 요청 실패: {exc}")
            self._publish_move_status(False)
            return

        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal 거절됨")
            self._publish_move_status(False)
            return

        with self._goal_lock:
            self._goal_handle = goal_handle
        self.get_logger().info("Nav2 goal 수락됨")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut: self._on_goal_result(
                fut,
                goal_lat=goal_lat,
                goal_lon=goal_lon,
                goal_request_id=goal_request_id,
            )
        )

    def _on_goal_result(
        self, future, goal_lat: float, goal_lon: float, goal_request_id: int
    ) -> None:  # noqa: ANN001
        with self._goal_lock:
            self._goal_handle = None
        try:
            result = future.result()
            status = int(result.status)
        except Exception as exc:
            self.get_logger().error(f"goal 결과 수신 실패: {exc}")
            self._publish_move_status(False)
            return

        if status == 4:
            self.get_logger().info("목표 도달 완료")
            self._publish_move_status(True)
            payload = self._build_command_status_payload(
                command_failed=False,
                reason="목표 지점에 도달했습니다.",
                start_lat=self.origin_lat,
                start_lon=self.origin_lon,
                goal_lat=goal_lat,
                goal_lon=goal_lon,
            )
            reached_msg = String()
            reached_msg.data = json.dumps(payload, ensure_ascii=False)
            self.reached_pub.publish(reached_msg)
        else:
            self.get_logger().warn(f"goal 종료 상태 코드: {status}")
            self._publish_move_status(False)

    def _publish_origin_marker(self) -> None:
        yaw = yaw_from_north_cw_deg(self.heading_deg)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        marker = Marker()
        marker.header.frame_id = self.path_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "enu_origin"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.2
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 1.0
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.origin_marker_pub.publish(marker)

        text_marker = Marker()
        text_marker.header.frame_id = self.path_frame
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "enu_origin"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 1.0
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.35
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = (
            "anchor(lat,lon,hdg)=("
            f"{format_float_full_precision(self.origin_lat)}, "
            f"{format_float_full_precision(self.origin_lon)}, "
            f"{self.heading_deg:.2f})"
        )
        self.origin_marker_pub.publish(text_marker)

    def _reset_mission_enu_origin(self) -> None:
        # mission anchor가 바뀌면 ENU 원점을 해당 anchor로 재설정한다.
        self._anchor_history_points = []
        self._anchor_history_reference_lat = float(self._mission_anchor_lat)
        self._anchor_history_reference_lon = float(self._mission_anchor_lon)
        self._publish_current_enu_from_anchor()

    def _record_anchor_history(self, lat: float, lon: float) -> None:
        if (
            self._anchor_history_reference_lat is None
            or self._anchor_history_reference_lon is None
        ):
            self._anchor_history_reference_lat = float(lat)
            self._anchor_history_reference_lon = float(lon)
        east, north = geodetic_to_enu(
            lat,
            lon,
            float(self._anchor_history_reference_lat),
            float(self._anchor_history_reference_lon),
        )
        point = Point()
        point.x = float(east)
        point.y = float(north)
        point.z = 0.05
        self._anchor_history_points.append(point)
        if len(self._anchor_history_points) > 2000:
            self._anchor_history_points.pop(0)
        self._publish_anchor_history_marker()

    def _publish_anchor_history_marker(self) -> None:
        history_marker = Marker()
        history_marker.header.frame_id = self.path_frame
        history_marker.header.stamp = self.get_clock().now().to_msg()
        history_marker.ns = "enu_anchor_history"
        history_marker.id = 2
        history_marker.type = Marker.LINE_STRIP
        history_marker.action = Marker.ADD
        history_marker.pose.orientation.w = 1.0
        history_marker.scale.x = 0.12
        history_marker.color.r = 1.0
        history_marker.color.g = 0.6
        history_marker.color.b = 0.1
        history_marker.color.a = 1.0
        history_marker.points = list(self._anchor_history_points)
        self.origin_marker_pub.publish(history_marker)

    def _publish_current_enu_from_anchor(self) -> None:
        if (
            self._mission_anchor_lat is None
            or self._mission_anchor_lon is None
            or self._latest_ligo_lat is None
            or self._latest_ligo_lon is None
        ):
            return
        east, north = geodetic_to_enu(
            self._latest_ligo_lat,
            self._latest_ligo_lon,
            float(self._mission_anchor_lat),
            float(self._mission_anchor_lon),
        )
        msg = PointStamped()
        msg.header.frame_id = self.mission_enu_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(east)
        msg.point.y = float(north)
        msg.point.z = 0.0
        self.current_enu_pub.publish(msg)

    def _on_map(self, msg: OccupancyGrid) -> None:
        if self._map_meta is None:
            self._map_meta = {
                "resolution": float(msg.info.resolution),
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
            }

    def _publish_move_status(self, success: bool) -> None:
        msg = Bool()
        msg.data = bool(success)
        self.move_status_pub.publish(msg)

    def _build_command_status_payload(
        self,
        command_failed: bool,
        reason: str,
        start_lat: float | None,
        start_lon: float | None,
        goal_lat: float,
        goal_lon: float,
    ) -> dict:
        return {
            "command_failed": bool(command_failed),
            "reason": str(reason),
            "start": {
                "lat": start_lat,
                "lon": start_lon,
            },
            "goal": {
                "lat": goal_lat,
                "lon": goal_lon,
            },
            "timestamp_unix": time.time(),
        }

    def _publish_goal_received_ack(
        self,
        goal_lat: float,
        goal_lon: float,
        command_failed: bool = False,
        reason: str = "",
    ) -> None:
        start_lat = None
        start_lon = None
        if self._mission_anchor_lat is not None and self._mission_anchor_lon is not None:
            start_lat = float(self._mission_anchor_lat)
            start_lon = float(self._mission_anchor_lon)
        payload = self._build_command_status_payload(
            command_failed=command_failed,
            reason=reason,
            start_lat=start_lat,
            start_lon=start_lon,
            goal_lat=goal_lat,
            goal_lon=goal_lon,
        )
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.goal_received_pub.publish(msg)

    def _publish_current_heading(self, heading_deg: float) -> None:
        msg = Float64()
        msg.data = float(heading_deg)
        self.current_heading_pub.publish(msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2RosController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
