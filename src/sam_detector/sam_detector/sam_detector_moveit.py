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

BUFFER_SIZE = 1

class SAMDetectorMoveIt(Node):

    def __init__(self, moveit: MoveItPy):
        super().__init__("sam_detector_moveit")

        self.logger = self.get_logger()
        self.moveit = moveit
        self.arm = self.moveit.get_planning_component("ar_manipulator")
        self.gripper = self.moveit.get_planning_component("ar_gripper")

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.subscription = self.create_subscription(
            TransformStamped,
            "/sam_detector/tf",
            self.listener_callback,
            1)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.is_running_command = True

        self.move_to_configuration("home")
        #time.sleep(2.0)
        self.open_gripper()
        #time.sleep(2.0)
        self.close_gripper()
        self.is_running_command = False

        self.transform_buffer = []
        

    def listener_callback(self, msg: Point):
        
        # self._logger.info(f"GOT MSG {msg}")
        if self.is_running_command:
            self._logger.info(f"Already running command.")
            return
        
        self.is_running_command = True

        if len(self.transform_buffer) >= BUFFER_SIZE:
            self.transform_buffer.pop(0)
            
        self.transform_buffer.append(msg)

        # Average the movement in the buffer, if it is stable then we try to move to it
        x_total = 0
        y_total = 0
        z_total = 0
        for val in self.transform_buffer:
            x_total += val.transform.translation.x
            y_total += val.transform.translation.y
            z_total += val.transform.translation.z

        x_avg = x_total / len(self.transform_buffer)
        y_avg = y_total / len(self.transform_buffer)
        z_avg = z_total / len(self.transform_buffer)

        VARIANCE = 0.005

        self.get_logger().info(f"{x_avg} -> {msg.transform.translation.x}")
        if len(self.transform_buffer) >= BUFFER_SIZE \
            and abs(x_avg - msg.transform.translation.x) < VARIANCE \
            and abs(y_avg - msg.transform.translation.y) < VARIANCE\
            and abs(z_avg - msg.transform.translation.z) < VARIANCE:
            self.pick_and_place()

        self.is_running_command = False
    
    def buffer_avg(self, buffer):
        total = 0
        for val in buffer:
            total += val
        
        return total / len(buffer)

    def generate_pose_goal(self, pose: PoseStamped):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose = pose
        return pose_goal

    def pick_and_place(self):
        GRIPPER_LENGTH = 0.06  # The robot model calculates at the base of the gipper, we need to offset to the gripper jaws
        try:
            target_object_transform = self.tf_buffer.lookup_transform("base_link", "gripper_target_object",
                                                    Time())
            
            bowl_transform = self.tf_buffer.lookup_transform("base_link", "detected_bowl",
                                                    Time())
            
            target_object_transform.child_frame_id = "gripper_target_object_moveit"
            self.tf_static_broadcaster.sendTransform(target_object_transform)
        
            self.arm.set_start_state_to_current_state()

            target_object_pose = Pose()
            target_object_pose.position.x = target_object_transform.transform.translation.x
            target_object_pose.position.y = target_object_transform.transform.translation.y
            target_object_pose.position.z = target_object_transform.transform.translation.z + 0.2 # Initial position ABOVE the object

            target_object_pose.orientation.w = target_object_transform.transform.rotation.w
            target_object_pose.orientation.x = target_object_transform.transform.rotation.x
            target_object_pose.orientation.y = target_object_transform.transform.rotation.y
            target_object_pose.orientation.z = target_object_transform.transform.rotation.z

            
            detected_bowl_pose = Pose()
            detected_bowl_pose.position.x = bowl_transform.transform.translation.x
            detected_bowl_pose.position.y = bowl_transform.transform.translation.y
            detected_bowl_pose.position.z = bowl_transform.transform.translation.z + 0.25  # Some extra clearance above the bowl
            
            detected_bowl_pose.orientation.w = bowl_transform.transform.rotation.w
            detected_bowl_pose.orientation.x = bowl_transform.transform.rotation.x
            detected_bowl_pose.orientation.y = bowl_transform.transform.rotation.y
            detected_bowl_pose.orientation.z = bowl_transform.transform.rotation.z
            

            self.logger.info("Move above object")
            self.arm.set_start_state_to_current_state()
            if self.move_gripper_to_pose_goal(self.generate_pose_goal(target_object_pose)):
              self.open_gripper()

              target_object_pose.position.z = target_object_transform.transform.translation.z + GRIPPER_LENGTH
              self.logger.info("Move around object")
              self.arm.set_start_state_to_current_state()
              if self.move_gripper_to_pose_goal(self.generate_pose_goal(target_object_pose)):
              
                time.sleep(0.2)
                self.close_gripper()
                time.sleep(0.2)
                self.logger.info("Move away from object")
                
                self.arm.set_start_state_to_current_state()
                target_object_pose.position.z = target_object_transform.transform.translation.z + 0.25
                if self.move_gripper_to_pose_goal(self.generate_pose_goal(target_object_pose)):
                  
                  self.logger.info("Move to bowl")
                  self.arm.set_start_state_to_current_state()
                  if self.move_gripper_to_pose_goal(self.generate_pose_goal(detected_bowl_pose)):
                    self.open_gripper()

                    time.sleep(0.2)
                    self.close_gripper()

            self.move_to_configuration("home")
        except Exception as ex:
            print(ex)

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
