import rclpy
import requests
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState


class MinimalSubscriber(Node):
    base_url = "http://10.1.1.74/"
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.get_logger().info('Starting subscriber...')
        self.change_j2_led_state(state=False)
        self.change_j5_led_state(state=False)

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    last_state = JointState()
    is_moving = False

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.position)
        if len(self.last_state.position) == 0:
            # Initial message, ignore
            self.last_state = msg
            return

        found_change = False
        for index, position in enumerate(msg.position):

            if self.last_state.position[index] != position:
                found_change = True

        if found_change and not self.is_moving:
            self.get_logger().info("ROBOT MOVING!")
            self.change_j2_led_state(state=True)
            self.change_j5_led_state(state=True)
        elif not found_change and self.is_moving:
            self.get_logger().info("ROBOT STOPPED!")
            self.change_j2_led_state(state=False)
            self.change_j5_led_state(state=False)
        
        self.is_moving = found_change

        self.last_state = msg
    
    def change_j2_led_state(self, state=True):
        url = self.base_url + 'light/robot_arm_ws2812_j2'
        if state:
            url += '/turn_on?effect=red_wipe'
            #url += '/turn_on?effect=Rainbow'
        else:
            url += '/turn_off'

        x = requests.post(url)

    def change_j5_led_state(self, state=True):
        url = self.base_url + 'light/robot_arm_ws2812_j5'
        if state:
            #url += '/turn_on?effect=red_wipe'
            url += '/turn_on?effect=Rainbow'
        else:
            url += '/turn_off'

        x = requests.post(url)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()