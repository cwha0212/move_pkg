from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile
import yaml


def _load_map_settings_from_move_pkg_config(config_path: str) -> tuple[float, float]:
    default_extent = 400.0
    default_resolution = 20.0
    try:
        with open(config_path, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file) or {}
        map_cfg = (
            config.get("nav2_ros_controller", {})
            .get("ros__parameters", {})
            .get("map", {})
        )
        resolution = float(map_cfg.get("resolution_m_per_cell", default_resolution))
        extent = float(map_cfg.get("extent_m", default_extent))
        return extent, resolution
    except Exception:
        return default_extent, default_resolution


def _load_nav2_controller_frequency_hz(config_path: str) -> float:
    default_hz = 20.0
    try:
        with open(config_path, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file) or {}
        nav_cfg = (
            config.get("nav2_ros_controller", {})
            .get("ros__parameters", {})
            .get("nav", {})
        )
        return float(nav_cfg.get("controller_frequency_hz", default_hz))
    except Exception:
        return default_hz


def _load_rotation_shim_settings(config_path: str) -> dict:
    defaults = {
        "enabled": False,
        "angular_dist_threshold": 0.785,
        "forward_sampling_distance": 0.5,
        "rotate_to_heading_angular_vel": 1.2,
        "max_angular_accel": 3.2,
        "simulate_ahead_time": 1.0,
        "rotate_to_goal_heading": True,
        "primary_controller": "dwb_core::DWBLocalPlanner",
    }
    try:
        with open(config_path, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file) or {}
        nav_cfg = (
            config.get("nav2_ros_controller", {})
            .get("ros__parameters", {})
            .get("nav", {})
        )
        shim_cfg = nav_cfg.get("rotation_shim", {})
        return {
            "enabled": bool(nav_cfg.get("use_rotation_shim", defaults["enabled"])),
            "angular_dist_threshold": float(
                shim_cfg.get(
                    "angular_dist_threshold_rad", defaults["angular_dist_threshold"]
                )
            ),
            "forward_sampling_distance": float(
                shim_cfg.get(
                    "forward_sampling_distance_m", defaults["forward_sampling_distance"]
                )
            ),
            "rotate_to_heading_angular_vel": float(
                shim_cfg.get(
                    "rotate_to_heading_angular_vel",
                    defaults["rotate_to_heading_angular_vel"],
                )
            ),
            "max_angular_accel": float(
                shim_cfg.get("max_angular_accel", defaults["max_angular_accel"])
            ),
            "simulate_ahead_time": float(
                shim_cfg.get("simulate_ahead_time_sec", defaults["simulate_ahead_time"])
            ),
            "rotate_to_goal_heading": bool(
                shim_cfg.get(
                    "rotate_to_goal_heading", defaults["rotate_to_goal_heading"]
                )
            ),
            "primary_controller": str(
                shim_cfg.get("primary_controller", defaults["primary_controller"])
            ),
        }
    except Exception:
        return defaults


def _load_nav2_frame_settings(config_path: str) -> dict:
    defaults = {
        "global_frame": "map",
        "odom_frame": "odom",
        "robot_base_frame": "base_link",
    }
    try:
        with open(config_path, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file) or {}
        frames_cfg = (
            config.get("nav2_ros_controller", {})
            .get("ros__parameters", {})
            .get("nav", {})
            .get("frames", {})
        )
        return {
            "global_frame": str(frames_cfg.get("global_frame", defaults["global_frame"])),
            "odom_frame": str(frames_cfg.get("odom_frame", defaults["odom_frame"])),
            "robot_base_frame": str(
                frames_cfg.get("robot_base_frame", defaults["robot_base_frame"])
            ),
        }
    except Exception:
        return defaults


def _build_runtime_map_yaml(
    base_map_yaml_path: str, map_extent: float, map_resolution: float
) -> str:
    with open(base_map_yaml_path, "r", encoding="utf-8") as file:
        map_yaml = yaml.safe_load(file) or {}
    pixels = max(2, int(round(map_extent / max(map_resolution, 1e-6))))
    runtime_image_path = os.path.join(
        tempfile.gettempdir(), "move_pkg_free_space_runtime.pgm"
    )
    with open(runtime_image_path, "wb") as file:
        header = f"P5\n{pixels} {pixels}\n255\n".encode("ascii")
        file.write(header)
        file.write(bytes([254]) * (pixels * pixels))
    map_yaml["image"] = runtime_image_path
    map_yaml["resolution"] = float(map_resolution)
    map_yaml["origin"] = [
        -0.5 * float(map_extent),
        -0.5 * float(map_extent),
        0.0,
    ]
    runtime_map_path = os.path.join(
        tempfile.gettempdir(),
        "move_pkg_free_space_runtime.yaml",
    )
    with open(runtime_map_path, "w", encoding="utf-8") as file:
        yaml.safe_dump(map_yaml, file, sort_keys=False)
    return runtime_map_path


def _build_runtime_nav2_params_yaml(
    base_nav2_params_path: str,
    controller_frequency_hz: float,
    shim_settings: dict,
    frame_settings: dict,
) -> str:
    with open(base_nav2_params_path, "r", encoding="utf-8") as file:
        nav2_params = yaml.safe_load(file) or {}
    global_frame = str(frame_settings.get("global_frame", "map"))
    odom_frame = str(frame_settings.get("odom_frame", "odom"))
    robot_base_frame = str(frame_settings.get("robot_base_frame", "base_link"))

    amcl_cfg = nav2_params.setdefault("amcl", {}).setdefault("ros__parameters", {})
    amcl_cfg["global_frame_id"] = global_frame
    amcl_cfg["odom_frame_id"] = odom_frame
    amcl_cfg["base_frame_id"] = robot_base_frame

    bt_cfg = nav2_params.setdefault("bt_navigator", {}).setdefault("ros__parameters", {})
    bt_cfg["global_frame"] = global_frame
    bt_cfg["robot_base_frame"] = robot_base_frame

    controller_server_cfg = nav2_params.setdefault("controller_server", {}).setdefault(
        "ros__parameters", {}
    )
    controller_server_cfg["controller_frequency"] = float(controller_frequency_hz)
    follow_path_cfg = controller_server_cfg.setdefault("FollowPath", {})
    if bool(shim_settings.get("enabled", False)):
        follow_path_cfg["plugin"] = "nav2_rotation_shim_controller::RotationShimController"
        follow_path_cfg["primary_controller"] = str(
            shim_settings.get("primary_controller", "dwb_core::DWBLocalPlanner")
        )
        follow_path_cfg["angular_dist_threshold"] = float(
            shim_settings.get("angular_dist_threshold", 0.785)
        )
        follow_path_cfg["forward_sampling_distance"] = float(
            shim_settings.get("forward_sampling_distance", 0.5)
        )
        follow_path_cfg["rotate_to_heading_angular_vel"] = float(
            shim_settings.get("rotate_to_heading_angular_vel", 1.2)
        )
        follow_path_cfg["max_angular_accel"] = float(
            shim_settings.get("max_angular_accel", 3.2)
        )
        follow_path_cfg["simulate_ahead_time"] = float(
            shim_settings.get("simulate_ahead_time", 1.0)
        )
        follow_path_cfg["rotate_to_goal_heading"] = bool(
            shim_settings.get("rotate_to_goal_heading", True)
        )
    else:
        follow_path_cfg["plugin"] = "dwb_core::DWBLocalPlanner"
        for key in (
            "primary_controller",
            "angular_dist_threshold",
            "forward_sampling_distance",
            "rotate_to_heading_angular_vel",
            "max_angular_accel",
            "simulate_ahead_time",
            "rotate_to_goal_heading",
        ):
            if key in follow_path_cfg:
                del follow_path_cfg[key]

    planner_cfg = nav2_params.setdefault("planner_server", {}).setdefault(
        "ros__parameters", {}
    )
    planner_cfg["global_frame"] = global_frame
    planner_cfg["robot_base_frame"] = robot_base_frame

    behavior_cfg = nav2_params.setdefault("behavior_server", {}).setdefault(
        "ros__parameters", {}
    )
    behavior_cfg["global_frame"] = global_frame
    behavior_cfg["robot_base_frame"] = robot_base_frame

    global_costmap_cfg = (
        nav2_params.setdefault("global_costmap", {})
        .setdefault("global_costmap", {})
        .setdefault("ros__parameters", {})
    )
    global_costmap_cfg["global_frame"] = global_frame
    global_costmap_cfg["robot_base_frame"] = robot_base_frame

    local_costmap_cfg = (
        nav2_params.setdefault("local_costmap", {})
        .setdefault("local_costmap", {})
        .setdefault("ros__parameters", {})
    )
    local_costmap_cfg["global_frame"] = odom_frame
    local_costmap_cfg["robot_base_frame"] = robot_base_frame
    runtime_nav2_params_path = os.path.join(
        tempfile.gettempdir(),
        "move_pkg_nav2_runtime_params.yaml",
    )
    with open(runtime_nav2_params_path, "w", encoding="utf-8") as file:
        yaml.safe_dump(nav2_params, file, sort_keys=False)
    return runtime_nav2_params_path


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    move_pkg_share = get_package_share_directory("move_pkg")

    default_move_pkg_params = os.path.join(move_pkg_share, "config", "nav2_ros.yaml")
    base_nav2_params = os.path.join(nav2_bringup_dir, "params", "nav2_params.yaml")
    base_map_yaml = os.path.join(move_pkg_share, "maps", "free_space_400m.yaml")
    map_extent, map_resolution = _load_map_settings_from_move_pkg_config(
        default_move_pkg_params
    )
    controller_frequency_hz = _load_nav2_controller_frequency_hz(
        default_move_pkg_params
    )
    rotation_shim_settings = _load_rotation_shim_settings(default_move_pkg_params)
    nav2_frame_settings = _load_nav2_frame_settings(default_move_pkg_params)
    default_nav2_params = _build_runtime_nav2_params_yaml(
        base_nav2_params,
        controller_frequency_hz,
        rotation_shim_settings,
        nav2_frame_settings,
    )
    default_map = _build_runtime_map_yaml(base_map_yaml, map_extent, map_resolution)

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    move_pkg_params_file = LaunchConfiguration("move_pkg_params_file")
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")
    use_simulation = LaunchConfiguration("use_simulation")
    rviz_config = LaunchConfiguration("rviz_config")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "map": map_yaml,
            "autostart": autostart,
        }.items(),
    )

    controller = Node(
        package="move_pkg",
        executable="nav2_ros_controller",
        name="nav2_ros_controller",
        output="screen",
        parameters=[move_pkg_params_file],
    )
    virtual_robot = Node(
        package="move_pkg",
        executable="virtual_robot_from_cmdvel",
        name="virtual_robot_from_cmdvel",
        output="screen",
        parameters=[move_pkg_params_file],
        condition=IfCondition(use_simulation),
    )
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=IfCondition(use_simulation),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_move_pkg",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument(
                "move_pkg_params_file",
                default_value=default_move_pkg_params,
            ),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_simulation", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(move_pkg_share, "rviz", "nav2_ros.rviz"),
            ),
            bringup,
            map_to_odom_tf,
            controller,
            virtual_robot,
            rviz_node,
        ]
    )
