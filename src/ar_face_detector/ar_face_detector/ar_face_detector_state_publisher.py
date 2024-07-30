#!/usr/bin/env python3

# generic ros libraries
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
import time
from geometry_msgs.msg import Point

from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from sensor_msgs.msg import JointState

JOINT_NAMES = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']

class ArFaceDetectorStatePublisher(Node):

    def __init__(self):
        super().__init__("ar_face_detector_sp")

        self.logger = self.get_logger()
        publish_topic = "/joint_trajectory_controller/joint_trajectory"

        self.state_listener = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_listener_callback,
            10)

        self.trajectory_publisher = self.create_publisher(JointTrajectory,publish_topic, 10)

        self.point_listener = self.create_subscription(
            Point,
            "/ar_face_detector/location",
            self.point_listener_callback,
            1)
        
        self.goal_positions = {}
        self.joint_positions = {}

        self.at_goal = True
        
        for joint in JOINT_NAMES:
            self.goal_positions[joint] = 0.0
            self.joint_positions[joint] = 0.0

        self.move_to_goal_positions()
        time.sleep(5)

    def state_listener_callback(self, msg):
        # print(f"state {msg}")
        joints = msg.name
        positions = msg.position

        positions_mapped = {}
        for joint in JOINT_NAMES:
            idx = joints.index(joint)
            pos = positions[idx]
            positions_mapped[joint] = pos
        
        self.joint_positions = positions_mapped

        self.at_goal = self.check_goal_reached()

    def point_listener_callback(self, point: Point):
        if not self.at_goal:
            self.logger.info("Not at goal")
            return
        
        # We want to try and keep the face in the center
        face_x = point.x
        face_y = point.y
        face_z = point.z  # Width of face as percentage

        target_x_pos = 0.35  # 0.5 is center of frame
        target_y_pos = 0.35
        target_face_size = 0.2  # Percentage of frame size
        
        dead_zone = 0.08
        dead_zone_z = 0.04

        x_scale = face_z * 2.5  # Scale the movement based on how close the face is. The closer, the faster the moves.
        y_scale = face_z * 1
        # x_scale = 0.5

        print(f"Face: {face_x} {x_scale}")

        x_change = (target_x_pos - face_x) * x_scale
        y_change = (target_y_pos - face_y) * y_scale
        z_change = (target_face_size - face_z) * y_scale

        #self.logger.info(f"Handle Face point: {x_change}, {z_change}, {y_change}")
        
        if face_x <= (target_x_pos - dead_zone) or face_x >= (target_x_pos + dead_zone):
          self.goal_positions["joint_1"] = self.joint_positions["joint_1"] - x_change
        
        
        if face_z <= (target_face_size - dead_zone_z) or face_z >= (target_face_size + dead_zone_z):
        #   self.goal_positions["joint_3"] = self.joint_positions["joint_3"] - z_change
          self.goal_positions["joint_2"] = self.joint_positions["joint_2"] + (z_change * 0.8)
          self.goal_positions["joint_3"] = self.joint_positions["joint_3"] - (z_change * 0.8)

        if face_y <= (target_y_pos - dead_zone) or face_y >= (target_y_pos + dead_zone):
          self.goal_positions["joint_2"] = self.joint_positions["joint_2"] - (y_change * 0.5)
          self.goal_positions["joint_3"] = self.joint_positions["joint_3"] - (y_change * 0.5)


        # print(f"Goal: {self.goal_positions}")

        self.move_to_goal_positions()

    def move_to_goal_positions(self):
        if not self.at_goal:
            return

        # print("move_to_goal", self.goal_positions)
        trajectory_msg = JointTrajectory()
        point = JointTrajectoryPoint()
        
        trajectory_msg.joint_names = []
        point.positions = []
        for _, joint in enumerate(self.goal_positions):
            trajectory_msg.joint_names.append(joint)
            point.positions.append(self.goal_positions[joint])

        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)

    def check_goal_reached(self):
        DIFF_MAX = 0.5
        for joint in JOINT_NAMES:
            if not joint in self.joint_positions:
                return True
            if not joint in self.goal_positions:
                return True
            
            diff = self.joint_positions[joint] - self.goal_positions[joint]
            
            if abs(diff) > DIFF_MAX:
                #self.logger.info(f"BAD DIFF {joint} {diff}")
                return False
        
        return True
            
        

def main():
    rclpy.init()
    node = ArFaceDetectorStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
