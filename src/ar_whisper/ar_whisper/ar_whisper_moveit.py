#!/usr/bin/env python3

# generic ros libraries
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
import time

# moveit python library
import tf2_ros
import transforms3d
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from moveit.core.robot_state import RobotState
from ar_interfaces.msg import ARWhisperCommand, ARWhisperCommands


class ArWhisperMoveIt(Node):

    def __init__(self, moveit: MoveItPy):
        super().__init__("ar_moveit_test")

        self.logger = self.get_logger()
        self.moveit = moveit
        self.arm = self.moveit.get_planning_component("ar_manipulator")
        self.gripper = self.moveit.get_planning_component("ar_gripper")

        self.subscription = self.create_subscription(
            ARWhisperCommands,
            '/ar_whisper_commands',
            self.listener_callback,
            10)

    def listener_callback(self, msg: ARWhisperCommands):
        self.logger.info(f"GOT MSG {msg}")

        for command in msg.commands:
            if command.gripper_state == "open":
                self.open_gripper()
            elif command.gripper_state == "closed":
                self.close_gripper()

            if command.position_name == "home":
                self.move_home()
            else:
              self.pose_move(command)

    def open_gripper(self):
        self.logger.info("Open Gripper")
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="open")
        self.plan_and_execute(self.gripper)

    def close_gripper(self):
        self.logger.info("Close Gripper")
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="closed")
        self.plan_and_execute(self.gripper)

    def move_home(self):
        self.logger.info("Move Home")
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="home")
        self.plan_and_execute(self.arm)

    def pose_move(self, command: ARWhisperCommand):
        self.logger.info(f"Move To Pose: {command}")
        pose = self.get_current_pose()

        pose.position.x += command.translate_x
        pose.position.y += command.translate_y
        pose.position.z += command.translate_z

        quat = [
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]

        # RVIZ Colors: red: x, green: y blue: z 
        x, y, z = transforms3d.euler.quat2euler(quat)
        
        x += command.rotate_x
        y += command.rotate_y
        z += command.rotate_z

        new_quat = transforms3d.euler.euler2quat(x, y, z)
        pose.orientation.w = new_quat[0]
        pose.orientation.x = new_quat[1]
        pose.orientation.y = new_quat[2]
        pose.orientation.z = new_quat[3]

        self.logger.info(f'{pose}')

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose = pose

        # pose_goal.pose = self.home_pose
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_6")

        # plan to goal
        attempt = 0

        while attempt < 3:
            attempt += 1
            result = self.plan_and_execute(self.arm)
            if result:
                return
    

    def get_current_pose(self):
        robot_model = self.moveit.get_robot_model()
        robot_state = RobotState(robot_model)
        # robot_state.update()  # Seems to make things worse..
        return robot_state.get_pose("link_6")

    def plan_and_execute(self, planning_component):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.logger.info("Planning trajectory")
        plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            return True
        else:
            self.logger.error("Planning failed")
            return False

def main():
    rclpy.init()
    moveit = MoveItPy(node_name="moveit_py")
    node = ArWhisperMoveIt(moveit)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
