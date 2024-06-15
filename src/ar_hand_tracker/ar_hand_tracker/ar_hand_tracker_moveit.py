#!/usr/bin/env python3

# generic ros libraries
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
import time
from geometry_msgs.msg import Point, TransformStamped

# moveit python library
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from moveit.core.robot_state import RobotState
from tf2_geometry_msgs import do_transform_pose


class ArFaceDetectorMoveIt(Node):

    def __init__(self, moveit: MoveItPy):
        super().__init__("ar_moveit_test")

        self.logger = self.get_logger()
        self.moveit = moveit
        self.arm = self.moveit.get_planning_component("ar_manipulator")

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.subscription = self.create_subscription(
            TransformStamped,
            "/ar_hand_tracker/tf",
            self.listener_callback,
            1)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.last_pose = None
        self.is_running_command = True

        self.move_to_configuration("home")
        self.is_running_command = False

    def listener_callback(self, msg: Point):
        
        # self._logger.info(f"GOT MSG {msg}")
        if self.is_running_command:
            self._logger.info(f"Already running command.")
            return
        
        self.is_running_command = True

        self.handle_face_point(msg)

        self.is_running_command = False

    def handle_face_point(self, transform: TransformStamped):

        try:
            new_transform = self.tf_buffer.lookup_transform("base_link", "tracked_hand",
                                                    Time())
            
            new_transform.child_frame_id = "tracked_hand_2"
            self.tf_static_broadcaster.sendTransform(new_transform)
            
            print(new_transform)
        
        
            self.arm.set_start_state_to_current_state()
            pose = self.last_pose
            if pose is None:
                # get_current_pose() is flaky so we only query it if we don't have a 
                # successful moveit move yet. Otherwise, we reuse the last pose that was successful
                pose = self.get_current_pose()

            self.logger.info(f"Current pose: {pose}")
            
            pose.position.x = new_transform.transform.translation.x
            pose.position.y = new_transform.transform.translation.y
            pose.position.z = new_transform.transform.translation.z + 0.10
            
            pose.orientation.w = new_transform.transform.rotation.w
            pose.orientation.x = new_transform.transform.rotation.x
            pose.orientation.y = new_transform.transform.rotation.y
            pose.orientation.z = new_transform.transform.rotation.z

            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "base_link"
            pose_goal.pose = pose

            self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_6")

            result = self.plan_and_execute(self.arm)
            if result:
                self.last_pose = pose
            else:
                self.last_pose = None
        except Exception as ex:
            print(ex)

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
