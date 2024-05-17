import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from sensor_msgs.msg import JointState

# FROM: https://github.com/noshluk2/ROS2-Ultimate-learners-Repository/blob/main/bazu/bazu/trajectory_gen.py
class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        
        #self.goal_positions = [0.5,0.5,0.5,0.5,0.3,0.1]
        self.goal_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        #self.goal_positions = [-0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_to_goal_positions()
        

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


    
    def listener_callback(self, msg):
        joints = msg.name
        positions = msg.position

        idx_j1 = joints.index("joint_1")
        pos_j1 = positions[idx_j1]
        pos_j1 = round(pos_j1, 2)

        if self.is_moving and pos_j1 == self.goal_positions[0]:
            self.get_logger().info('J1: "%s"' % pos_j1)
            self.is_moving = False

            if pos_j1 == 0.0:
                self.goal_positions = [-0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.move_to_goal_positions()
            elif pos_j1 == -0.5:
                self.goal_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.move_to_goal_positions()


def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
