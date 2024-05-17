import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from sensor_msgs.msg import JointState
from .livewhisper import StreamHandler
import re
import math

# FROM: https://github.com/noshluk2/ROS2-Ultimate-learners-Repository/blob/main/bazu/bazu/trajectory_gen.py
class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"


        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        
        #self.goal_positions = [0.5,0.5,0.5,0.5,0.3,0.1]
        self.goal_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_positions = self.goal_positions
        
        #self.goal_positions = [-0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_to_goal_positions()
        

        self.running = True
        self.talking = False
        self.prompted = False
        self.handler = StreamHandler(self)
        self.handler.listen()

    is_moving = False
    def move_to_goal_positions(self):
        self.is_moving = True
        print("move_to_goal", self.goal_positions)
        bazu_trajectory_msg = JointTrajectory()
        bazu_trajectory_msg.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        ## creating a point
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=5)
        ## adding newly created point into trajectory message
        bazu_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(bazu_trajectory_msg)



    def analyze(self, input):  # This is the decision tree for the assistant
        command = input.lower().strip()

        print("WE GOT")
        print(command)

        if not command.startswith("move"):
            print("Move command not found. Ignoring")
            return

        parts = command.split(" ")
        if len(parts) == 3:
            # First part is "move"
            # Second part is distance
            # Third part is direction

            try:
                distance = re.sub(r'\W+', '', parts[1])  # Remove all non-alphanumeric
                direction = re.sub(r'\W+', '', parts[2])

                radians = math.radians(int(distance))

                print(radians)
                print(direction)

                if direction == 'left':
                    # Move joint 1    
                    self.goal_positions[0] -= radians
                elif direction == 'right':
                    # Move joint 1    
                    self.goal_positions[0] += radians
                
                elif direction == 'up':
                    # Move joint 1    
                    self.goal_positions[2] -= radians
                
                elif direction == 'down':
                    # Move joint 1    
                    self.goal_positions[2] += radians
                elif direction == 'forward':
                    # Move joint 1    
                    self.goal_positions[1] += radians
                
                elif direction.contains('back'):
                    # Move joint 1    
                    self.goal_positions[1] -= radians

                self.move_to_goal_positions()
            except Exception as ex:
                print(ex)
                pass


def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
