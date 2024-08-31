import rclpy
import requests
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import threading


class ESPHomeLEDS(Node):
    base_url = "http://10.1.1.93/"
    #base_url = "http://robot_arm_ws2812_leds.local/"
    face_found = False

    def __init__(self):
        super().__init__('esphome_leds')

        self.get_logger().info('Starting subscriber...')
        self.change_j1_led_state(state=False)
        self.change_j2_led_state(state=False)
        self.change_j5_led_state(state=False)

        self.point_listener = self.create_subscription(
            Point,
            "/ar_face_detector/location",
            self.listener_callback,
            1)
        
        self.off_timer = None

    def listener_callback(self, msg):
        # When we find a face, turn red
        if not self.face_found:
            self.turn_all_red()
            self.face_found = True
        
        if self.off_timer is not None:
            self.off_timer.cancel()
        # After 1 second of find a face turn green
        self.off_timer = threading.Timer(0.3, self.turn_all_green)
        self.off_timer.start()
        

    def turn_all_green(self):
        self.face_found = False
        self.change_j1_led_state(state=False)
        self.change_j2_led_state(state=False)
        self.change_j5_led_state(state=False)

    def turn_all_red(self):
        self.change_j1_led_state(state=True)
        self.change_j2_led_state(state=True)
        self.change_j5_led_state(state=True)

    def change_j1_led_state(self, state=True):
        url = self.base_url + 'light/robot_arm_ws2812_j1'
        if state:
            url += '/turn_on?effect=red_wipe'
            #url += '/turn_on?effect=Rainbow'
        else:
            url += '/turn_on?effect=green_wipe'
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
            url += '/turn_on?effect=green_wipe'
        try:
            x = requests.post(url)
        except:
            self.get_logger().error("Failed to communicate with ESP lights harware")

    def change_j5_led_state(self, state=True):
        url = self.base_url + 'light/robot_arm_ws2812_j5'
        if state:
            #url += '/turn_on?effect=red_wipe'
            url += '/turn_on?effect=red_wipe'
        else:
            url += '/turn_on?effect=green_wipe'

        try:
            x = requests.post(url)
        except:
            self.get_logger().error("Failed to communicate with ESP lights harware")

def main(args=None):
    rclpy.init(args=args)

    node = ESPHomeLEDS()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.change_j1_led_state(False)
        node.change_j2_led_state(False)
        node.change_j5_led_state(False)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()