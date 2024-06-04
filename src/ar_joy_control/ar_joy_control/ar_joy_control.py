import rclpy
import time
import requests
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy, JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand


class ArJoyControl(Node):
    base_url = "http://10.1.1.74/"

    def __init__(self):
        super().__init__('ar_joy_control')
        self.logger = self.get_logger()
    
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        self.gripper_client = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")

        self.gripper_client.wait_for_server()

        self.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        self.actual_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.goal_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_open = False
        self.j5_mode = False

        self.last_move_ts = time.time()

        self.change_j1_led_state(False)
        self.change_j2_led_state(False)
        self.change_j5_led_state(False)
        self.move_to_goal_positions()
        self.close_gripper()

        time.sleep(5.0)

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            1)
        
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_listener_callback,
            10)

    def listener_callback(self, msg: Joy):
        self.logger.info(f"Got msg: {msg}")

        lstick_x = msg.axes[0]
        lstick_y = msg.axes[1]
        l_trigger = msg.axes[2]

        rstick_x = msg.axes[3]
        rstick_y = msg.axes[4]
        r_trigger = msg.axes[5]

        dpad_x = msg.axes[6]
        dpad_y = msg.axes[7]

        btn_a = msg.buttons[0]
        btn_b = msg.buttons[1]
        btn_x = msg.buttons[2]
        btn_y = msg.buttons[3]

        should_move = False


        if btn_a == 1 and not self.gripper_open:
            self.gripper_open = True
            self.open_gripper()
        elif btn_a == 0 and self.gripper_open:
            self.gripper_open = False
            self.close_gripper()
        
        if btn_b == 1 and not self.j5_mode:
            self.j5_mode = True
            self.change_j5_led_state(True)
        elif btn_b == 0 and self.j5_mode:
            self.j5_mode = False
            self.change_j5_led_state(False)

        if abs(lstick_x) > 0.05:
            self.goal_positions[0] = self.actual_positions[0]
            self.goal_positions[0] -= (lstick_x / 5)
            should_move = True

        if abs(lstick_y) > 0.05:
            self.goal_positions[1] = self.actual_positions[1]
            self.goal_positions[1] -= (lstick_y / 5)
            should_move = True
        
        if abs(rstick_y) > 0.05:
            if self.j5_mode:
                self.goal_positions[4] = self.actual_positions[4]
                self.goal_positions[4] -= (rstick_y / 1)  # Is inverted
            else:
                self.goal_positions[2] = self.actual_positions[2]
                self.goal_positions[2] -= (rstick_y / 1)  # Is inverted
            should_move = True

        if abs(rstick_x) > 0.05:
            if self.j5_mode:
                self.goal_positions[5] = self.actual_positions[5]
                self.goal_positions[5] -= (rstick_x / 1)
            else:
                self.goal_positions[3] = self.actual_positions[3]
                self.goal_positions[3] -= (rstick_x / 2)
            should_move = True

        
        diff = time.time() - self.last_move_ts

        if should_move:
            # self.change_j1_led_state(True)
            # self.change_j2_led_state(True)
            if diff > 0.3:  # Throttle the move command
                print("MOVE")
                self.move_to_goal_positions()
                self.last_move_ts = time.time()
        else:
            self.goal_positions = self.actual_positions
            self.move_to_goal_positions()
            # self.change_j1_led_state(False)
            # self.change_j2_led_state(False)

    def joint_listener_callback(self, msg: JointState):
        # self.logger.info(f"Got joints: {msg}")

        for (index, name) in enumerate(self.joint_names):
            idx = msg.name.index(name)
            pos = msg.position[idx]
            self.actual_positions[index] = pos
        
        # print(self.actual_positions)

    def move_to_goal_positions(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        ## creating a point
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=1)
        ## adding newly created point into trajectory message
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)
    

    def open_gripper(self):
        cmd = GripperCommand.Goal()
        cmd.command.max_effort = 1.0
        cmd.command.position = -0.01
        self.gripper_client.send_goal_async(cmd)
    
    def close_gripper(self):
        cmd = GripperCommand.Goal()
        cmd.command.max_effort = 1.0
        cmd.command.position = 0.0
        self.gripper_client.send_goal_async(cmd)

    def change_j1_led_state(self, state=True):
        url = self.base_url + 'light/robot_arm_ws2812_j1'
        if state:
            url += '/turn_on?effect=green_wipe'
        else:
            url += '/turn_on?effect=Rainbow'
        try:
            x = requests.post(url)
        except:
            self.get_logger().error("Failed to communicate with ESP lights harware")

    def change_j2_led_state(self, state=True):
        url = self.base_url + 'light/robot_arm_ws2812_j2'
        if state:
            url += '/turn_on?effect=red_wipe'
            #url += '/turn_on?effect=Rainbow'
        else:
            url += '/turn_on?effect=Rainbow'
        try:
            x = requests.post(url)
        except:
            self.get_logger().error("Failed to communicate with ESP lights harware")

    def change_j5_led_state(self, state=True):
        url = self.base_url + 'light/robot_arm_ws2812_j5'
        if state:
            #url += '/turn_on?effect=red_wipe'
            url += '/turn_on?effect=blue_wipe'
        else:
            url += '/turn_on?effect=Rainbow'

        try:
            x = requests.post(url)
        except:
            self.get_logger().error("Failed to communicate with ESP lights harware")


def main(args=None):
    rclpy.init(args=args)
    node = ArJoyControl()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
