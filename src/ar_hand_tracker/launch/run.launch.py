import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Make MoveIt use simulation time. This is needed "+\
                "for trajectory planing in simulation.",
        ))

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("ar_description"), "urdf", "ar.urdf.xacro"]),
        " ",
        "name:=ar",
        " ",
        "include_gripper:=",
        "False",
    ])
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("ar_moveit_config"), "srdf", "ar.srdf.xacro"]),
        " ",
        "name:=ar",
        " ",
        "include_gripper:=",
        "False",
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "ar_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    robot_description_planning = {
        "robot_description_planning":
        load_yaml(
            "ar_hand_tracker",
            os.path.join("config", "joint_limits.yaml"),
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
        }
    }
    ompl_planning_yaml = load_yaml("ar_moveit_config",
                                   "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ar_moveit_config", "config/controllers.yaml")

    moveit_controllers = {
        "moveit_simple_controller_manager":
        controllers_yaml,
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.2,
            "allowed_goal_duration_margin": 0.5,
            "allowed_start_tolerance": 0.9,
        }
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    params_dict = {}
    params_dict.update(robot_description)
    params_dict.update(robot_description_semantic)
    params_dict.update(robot_description_planning)
    params_dict.update(robot_description_kinematics)
    params_dict.update(ompl_planning_pipeline_config)
    params_dict.update(
        load_yaml(
            "ar_hand_tracker",
            os.path.join("config", "moveit_py_parameters.yaml"),
        ))
    params_dict.update(moveit_controllers)
    params_dict.update(trajectory_execution)
    params_dict.update(planning_scene_monitor_parameters)
    params_dict.update(
        {
            "use_sim_time": use_sim_time
        }
    )

    realsense_args = {
        "initial_reset": "true",
        "enable_rgbd": "true",
        "enable_sync": "true",
        "align_depth.enable": "true",
        "enable_color": "true",
        "enable_depth": "true",
        # "depth_module.depth_profile": "640x480x15",
        # "depth_module.infra_profile": "640x480x15",
        "clip_distance": "1.5",
        # "pointcloud.pointcloud_qos": "SENSOR_DATA",
        "pointcloud.enable": "true",
        "pointcloud.allow_no_texture_points": "false",
        # "pointcloud.ordered_pc": "true",
        # "pointcloud.stream_filter": "1"
    }
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("realsense2_camera"),
                         "launch", "rs_launch.py")
        ]),
        launch_arguments=realsense_args.items())

    moveit_node = Node(
        package="ar_hand_tracker",
        executable="ar_hand_tracker_moveit",
        name="moveit_py",
        output="screen",
        parameters=[params_dict],
    )

    tracker_node = Node(
        package="ar_hand_tracker",
        executable="ar_hand_tracker",
        output="screen",
    )

    esphome_leds_node = Node(
        package="esphome_leds",
        executable="esphome_leds",
        output="screen",
    )
    
    return LaunchDescription([
      realsense_node,
      moveit_node,
      esphome_leds_node,
      tracker_node
    ])
