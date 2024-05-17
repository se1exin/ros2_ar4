#!/usr/bin/env python3

# generic ros libraries
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import time

# moveit python library
import tf2_ros
import transforms3d
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from tf2_geometry_msgs import do_transform_pose


class ArMoveItTest(Node):

    def __init__(self, moveit: MoveItPy):
        super().__init__("ar_moveit_test")
        self.logger = self.get_logger()
        self.moveit = moveit
        self.arm = self.moveit.get_planning_component("ar_manipulator")
        self.gripper = self.moveit.get_planning_component("ar_gripper")
        

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.logger.info("SLEEPING")
        time.sleep(5)
        self.logger.info("RUNNING TEST MOVE")
        
        self.move_home()

        self.robot_model = self.moveit.get_robot_model()
        self.robot_state = RobotState(self.robot_model)
        self.home_pose = self.robot_state.get_pose("link_6")
        #self.logger.info(self.home_pose.orientation)
        #self.logger.info(self.home_pose.position)
        #self.test_random_move()

        self.test_pose_move()

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

    def test_random_move(self):
        self.logger.info("Move To Random Position")
        # randomize the robot state
        self.robot_state.set_to_random_positions()
        

        # set plan start state to current state
        self.arm.set_start_state_to_current_state()

        # set goal state to the initialized robot state
        self.logger.info("Set goal state to the initialized robot state")
        self.arm.set_goal_state(robot_state=self.robot_state)

        # plan to goal
        self.plan_and_execute(self.arm)
    
    def test_pose_move(self):
        self.logger.info("Move To Pose")
        self.arm.set_start_state_to_current_state()

        # pose = self.robot_state.get_pose("link_6")
        # transform = self.tf_buffer.lookup_transform(target_frame="link_6", source_frame="base_link", time=Time())
        # transformed_pose = do_transform_pose(pose, transform)
        # transformed_pose.position.z += 0.05
        
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.orientation.w = -1.0
        pose_goal.pose.orientation.y = -1.0
        pose_goal.pose.orientation.z = -1.0
        
        pose_goal.pose.position.x = 0.24
        pose_goal.pose.position.y = -0.25
        pose_goal.pose.position.z = 0.5

        # pose_goal.pose = self.home_pose

        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_6")

        # plan to goal
        self.plan_and_execute(self.arm)

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
        else:
            self.logger.error("Planning failed")


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="moveit_py")
    node = ArMoveItTest(moveit)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
