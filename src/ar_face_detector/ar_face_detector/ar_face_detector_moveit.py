#!/usr/bin/env python3

# generic ros libraries
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
import time
from geometry_msgs.msg import Point

# moveit python library
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from moveit.core.robot_state import RobotState


class ArFaceDetectorMoveIt(Node):

    def __init__(self, moveit: MoveItPy):
        super().__init__("ar_moveit_test")

        self.logger = self.get_logger()
        self.moveit = moveit
        self.arm = self.moveit.get_planning_component("ar_manipulator")

        self.subscription = self.create_subscription(
            Point,
            "/ar_face_detector/location",
            self.listener_callback,
            1)
        
        self.last_pose = None
        self.is_running_command = True

        self.move_to_configuration("facing_left")
        self.is_running_command = False

    def listener_callback(self, msg: Point):
        # self._logger.info(f"GOT MSG {msg}")
        if self.is_running_command:
            self._logger.info(f"Already running command.")
            return
        
        self.is_running_command = True

        self.handle_face_point(msg)

        self.is_running_command = False

    def handle_face_point(self, point):
        
        # We want to try and keep the face in the center
        face_x = point.x
        face_y = point.y
        face_z = point.z

        self.logger.info(f"Handle Face point: {face_x}, {face_y}, {face_z}")

        self.arm.set_start_state_to_current_state()
        pose = self.last_pose
        if pose is None:
            # get_current_pose() is flaky so we only query it if we don't have a 
            # successful moveit move yet. Otherwise, we reuse the last pose that was successful
            pose = self.get_current_pose()

        self.logger.info(f"Current pose: {pose}")

        x_change = (0.5 - face_x) / 2
        z_change = (0.5 - face_y) / 2
        y_change = (0.3 - face_z) * 1.5

        if face_x <= 0.45 or face_x >= 0.55:
          pose.position.x += x_change

        if face_y <= 0.45 or face_y >= 0.55:
          pose.position.z += z_change
        
        # if face_z < 0.24 or face_z > 0.26:
        #   pose.position.y -= y_change
        pose.position.y -= y_change

        
        self.logger.info(f"Face point Move To: {face_z} -> {y_change}")

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose = pose

        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_6")

        result = self.plan_and_execute(self.arm)
        if result:
            self.last_pose = pose
        else:
            self.last_pose = None

    def move_to_configuration(self, config_name):
        self.logger.info(f"Move to config {config_name}")
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=config_name)
        self.plan_and_execute(self.arm)
        self.last_pose = self.get_current_pose()


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
    node = ArFaceDetectorMoveIt(moveit)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
