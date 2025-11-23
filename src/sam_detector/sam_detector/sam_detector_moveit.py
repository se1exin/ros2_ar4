#!/usr/bin/env python3

# generic ros libraries
import threading
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
import time
from geometry_msgs.msg import Point, TransformStamped

# moveit python library
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from moveit.core.robot_state import RobotState
from tf2_geometry_msgs import do_transform_pose

class SAMDetectorMoveIt(Node):

    def __init__(self, moveit: MoveItPy):
        super().__init__("sam_detector_moveit")

        self.logger = self.get_logger()

        self.moveit = moveit
        self.arm = self.moveit.get_planning_component("ar_manipulator")
        self.gripper = self.moveit.get_planning_component("ar_gripper")

        self.is_running_command = True

        self.move_to_configuration("home")
        time.sleep(1.0)
        self.open_gripper()
        time.sleep(1.0)
        self.close_gripper()
        time.sleep(1.0)
        self.is_running_command = False


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.loop_timer = self.create_timer(1.0, self.loop_timer_callback)
    

    def loop_timer_callback(self):
        if self.is_running_command:
            self._logger.info(f"Already running command.")
            return
        
        self.is_running_command = True

        try:
            target_object_transform = self.tf_buffer.lookup_transform(
                target_frame="base_link",
                source_frame="gripper_target_object",
                time=Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            # Check transform age
            now = self.get_clock().now().to_msg()
            t = target_object_transform.header.stamp
            age = (now.sec + now.nanosec * 1e-9) - (t.sec + t.nanosec * 1e-9)
            if age > 5.0:
                self.get_logger().warn(f"Transform is too old: {age:.2f} seconds. Skipping.")
                self.is_running_command = False
                return
            target_object_transform.child_frame_id = "gripper_target_object_moveit"
            target_object_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(target_object_transform)

            time.sleep(0.2)  # Give some time for the transform to propagate
        except Exception as e:
            self.get_logger().error(f"Transform lookup failed in moveit node: {e}")
            self.is_running_command = False
            return
        
        # Run in a separate thread to not block the timer callback
        # Also because moveit seems to mess with the transform buffer
        threading.Thread(target=self.pick_and_place, args=(target_object_transform,)).start()
        # self.pick_and_place(target_object_transform)


    def generate_pose_goal(self, pose: PoseStamped):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose = pose
        return pose_goal

    def pick_and_place(self, target_object_transform):
        GRIPPER_LENGTH = 0.05  # The robot model calculates at the base of the gipper, we need to offset to the gripper jaws
        try:
            self.arm.set_start_state_to_current_state()

            target_object_pose = Pose()
            target_object_pose.position.x = target_object_transform.transform.translation.x
            target_object_pose.position.y = target_object_transform.transform.translation.y + GRIPPER_LENGTH + 0.08
            target_object_pose.position.z = target_object_transform.transform.translation.z

            target_object_pose.orientation.w = target_object_transform.transform.rotation.w
            target_object_pose.orientation.x = target_object_transform.transform.rotation.x
            target_object_pose.orientation.y = target_object_transform.transform.rotation.y
            target_object_pose.orientation.z = target_object_transform.transform.rotation.z

            if target_object_pose.position.z <= 0.05:
                target_object_pose.position.z = 0.05

            self.logger.info("Move above object")
            self.arm.set_start_state_to_current_state()
            if self.move_gripper_to_pose_goal(self.generate_pose_goal(target_object_pose)):
              time.sleep(1.0)
              self.open_gripper()

              target_object_pose.position.y = target_object_transform.transform.translation.y + GRIPPER_LENGTH
              self.logger.info("Move around object")
              self.arm.set_start_state_to_current_state()
              if self.move_gripper_to_pose_goal(self.generate_pose_goal(target_object_pose)):
              
                time.sleep(0.2)
                self.close_gripper()
                time.sleep(1.0)
                # self.logger.info("Move away from object")
                
                # self.arm.set_start_state_to_current_state()
                # target_object_pose.position.x = target_object_transform.transform.translation.x + 0.25
                # if self.move_gripper_to_pose_goal(self.generate_pose_goal(target_object_pose)):
                #     pass

            self.move_to_configuration("home")
            time.sleep(0.5)
            self.open_gripper()
            time.sleep(3)
            self.is_running_command = False

        except Exception as ex:
            self.get_logger().error(f"{ex}")
            self.is_running_command = False

    def move_gripper_to_pose_goal(self, pose_goal: PoseStamped):
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_6")

        return self.plan_and_execute(self.arm)

    def move_to_configuration(self, config_name):
        self.logger.info(f"Move to config {config_name}")
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=config_name)
        self.plan_and_execute(self.arm)


    def get_current_pose(self):
        robot_model = self.moveit.get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()  # Seems to make things worse..
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

def main():
    rclpy.init()
    moveit = MoveItPy(node_name="moveit_py")
    node = SAMDetectorMoveIt(moveit)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
