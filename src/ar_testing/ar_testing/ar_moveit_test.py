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
from tf2_geometry_msgs import do_transform_pose


class ArMoveItTest(Node):

    def __init__(self, moveit: MoveItPy):
        super().__init__("ar_moveit_test")
        self.target_frame = self.declare_parameter(
          'target_frame', 'base_link').get_parameter_value().string_value

        self.logger = self.get_logger()
        self.moveit = moveit
        self.arm = self.moveit.get_planning_component("ar_manipulator")
        self.gripper = self.moveit.get_planning_component("ar_gripper")
        
        

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.link_6_tf = None

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.logger.info("SLEEPING")
        time.sleep(2)
        self.logger.info("RUNNING TEST MOVE")
        
        self.move_home()
        #self.open_gripper()
        #self.test_random_move()
        #self.close_gripper()
        time.sleep(2)

        self.home_pose = self.get_current_pose()
        self.logger.info(f'{self.home_pose}')
        
        #return
        while True:
            time.sleep(2.0)
            self.test_pose_move()
    
    def on_timer(self):
        """
        Callback function.
        This function gets called at the specific time interval.
        """
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'link_6'
        
        try:
            now = rclpy.time.Time()
            self.link_6_tf = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
            self.get_logger().info(f"GOT TF: {self.link_6_tf}")
        except tf2_ros.TransformException as ex:
            self.link_6_tf = None
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        return

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
        robot_model = self.moveit.get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.set_to_random_positions()
        self.arm.set_start_state_to_current_state()
        self.logger.info("Set goal state to the initialized robot state")
        self.arm.set_goal_state(robot_state=robot_state)

        # plan to goal
        self.plan_and_execute(self.arm)
    
    def test_pose_move(self):
        self.logger.info("Move To Pose")
        self.arm.set_start_state_to_current_state()
        pose = self.get_current_pose()

        quat = [
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]

        # RVIZ Colors: red: x, green: y blue: z 
        x, y, z = transforms3d.euler.quat2euler(quat)
        self.logger.info(f'euler: {x}, {y}, {z}')
        x += 0.5
        # y += 0.1
        # z += 0.1

        new_quat = transforms3d.euler.euler2quat(x, y, z)

        pose.orientation.w = new_quat[0]
        pose.orientation.x = new_quat[1]
        pose.orientation.y = new_quat[2]
        pose.orientation.z = new_quat[3]

        # pose.position.x += 0.05
        pose.position.z += 0.05
        #pose.position.y += 0.05

        # pose = self.home_pose
        # pose.position.x += 0.05
        # pose.position.z += 0.05

        self.logger.info(f'{pose}')

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose = pose

        # pose_goal.pose = self.home_pose

        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_6")

        # plan to goal
        attempt = 0

        while attempt < 3:
            attempt += 1
            result = self.plan_and_execute(self.arm)
            if result:
                return
    
    def get_current_transform(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'link_6'
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
            self.get_logger().info(f"{trans}")
            return trans
        except tf2_ros.TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return None

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

    def plan_and_execute_old(
        self,
        planning_component,
        single_plan_parameters=None
        ):
        """A helper function to plan and execute a motion."""
        # plan to goal
        self.logger.info("Planning trajectory")

        multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
            self.moveit, ["ompl_rrtc"]
        )

        if multi_pipeline_plan_request_params is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_pipeline_plan_request_params
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
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
