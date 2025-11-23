import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

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
    tf_prefix = LaunchConfiguration("tf_prefix")

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for AR4 tf_tree",
        ))

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("annin_ar4_description"), "urdf", "ar.urdf.xacro"]),
        " ",
        "name:=ar",
        " ",
        "ar_model:=mk2",
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "include_gripper:=",
        "True",
    ])
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_moveit_config"), "srdf",
            "ar.srdf.xacro"
        ]),
        " ",
        "name:=mk2",
        " ",
        "ar_model:=mk2",
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "include_gripper:=",
        "True",
    ])
    
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "annin_ar4_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "annin_ar4_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    joint_limits = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("sam_detector"),
            "config/joint_limits.yaml"
        ]),
        allow_substs=True,
    )

    # Planning Configuration
    ompl_planning_yaml = load_yaml("annin_ar4_moveit_config",
                                   "config/ompl_planning.yaml")
    pilz_planning_yaml = load_yaml("annin_ar4_moveit_config",
                                   "config/pilz_planning.yaml")
    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl", "pilz"],
        "ompl": ompl_planning_yaml,
        "pilz": pilz_planning_yaml,
    }

    # Trajectory Execution Configuration
    moveit_controller_manager = {
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Trajectory Execution Configuration
    moveit_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("sam_detector"),
            "config/controllers.yaml"
        ]),
        allow_substs=True,
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        # Added due to https://github.com/moveit/moveit2_tutorials/issues/528
        "publish_robot_description_semantic": True,
    }


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
        # "pointcloud.enable": "true",
        # "pointcloud.allow_no_texture_points": "false",
        # "pointcloud.ordered_pc": "true",
        # "pointcloud.stream_filter": "1"
    }

    moveit_node = Node(
        package="sam_detector",
        executable="sam_detector_moveit",
        name="moveit_py",
        output="screen",
        parameters=[
            joint_limits,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
            trajectory_execution,
            load_yaml(
                "sam_detector",
                os.path.join("config", "moveit_py_parameters.yaml"),
            ),
            moveit_controller_manager,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {
                "use_sim_time": False,
            }
        ],
    )
    
    camera_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("sam_detector"), "urdf", "sam_camera.xacro"
        ])
    ])
    camera_description = {"robot_description": camera_description_content}

    camera_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="camera_state_publisher",
        namespace="camera",
        output="screen",
        parameters=[camera_description]
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("realsense2_camera"),
                         "launch", "rs_launch.py")
        ]),
        launch_arguments=realsense_args.items())

    tracker_node = Node(
        package="sam_detector",
        executable="sam_detector",
        output="screen",
    )

    
    return LaunchDescription(declared_arguments + [
      moveit_node,
      camera_state_publisher,
      realsense_node,
      #tracker_node
    ])
