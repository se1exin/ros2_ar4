import rclpy
import requests
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point
import threading

class NerfFirer(Node):
    base_url = "http://10.1.1.59/"
    def __init__(self):
        super().__init__('ar_nerf_firer')

        self.get_logger().info('Starting subscriber...')
        self.change_motor_state(state=False)
        self.change_trigger_state(state=False)

        self.point_listener = self.create_subscription(
            Point,
            "/ar_face_detector/location",
            self.point_listener_callback,
            1)

        self.motor_on_duration = 1.0  # Duration in seconds
        self.motor_off_duration = 0.5
        self.trigger_off_duration = 1.0
        self.trigger_on_duration = 0.5
        self.motor_timer = None
        self.trigger_start_timer = None
        self.trigger_stop_timer = None
        self.needs_call = False
        self.motor_state = False
        self.trigger_state = False
        self.face_found = False


    def point_listener_callback(self, point: Point):
      if self.motor_timer is not None:
        self.motor_timer.cancel()
      self.motor_timer = threading.Timer(self.motor_off_duration, self.stop_motor)
      self.motor_timer.start()
      self.face_found = True

      if self.motor_state is False:
        self.get_logger().info('Start motor!')
        self.change_motor_state(True)

      self.start_trigger_timer()

    def stop_motor(self):
      self.face_found = False
      self.get_logger().info('Stop motor!')
      self.stop_trigger()
      self.change_motor_state(False)
      self.cancel_trigger_timer()
    
    def start_trigger_timer(self):
        if self.trigger_start_timer is None:
          self.trigger_start_timer = threading.Timer(self.trigger_off_duration, self.start_trigger)
          self.trigger_start_timer.start()
    
    def cancel_trigger_timer(self):
        if self.trigger_start_timer is not None:
            self.trigger_start_timer.cancel()
            self.trigger_start_timer = None

    def start_trigger(self):
      self.get_logger().info('Start trigger!')
      self.change_trigger_state(True)

      self.trigger_stop_timer = threading.Timer(self.trigger_on_duration, self.stop_trigger)
      self.trigger_stop_timer.start()

    def stop_trigger(self):
      self.get_logger().info('Stop trigger!')
      self.change_trigger_state(False)
      self.trigger_start_timer = None
      self.trigger_stop_timer = None
    #   if self.face_found:
    #     self.start_trigger_timer()

    def change_motor_state(self, state=True):
        url = self.base_url + 'switch/nerf_start'
        if state:
            url += '/turn_on'
        else:
            url += '/turn_off'
        try:
            x = requests.post(url)
            self.motor_state = state
        except:
            self.get_logger().error("Failed to communicate")

    def change_trigger_state(self, state=True):
        url = self.base_url + 'switch/nerf_trigger'
        if state:
            url += '/turn_on'
        else:
            url += '/turn_off'
        try:
            x = requests.post(url)
        except:
            self.get_logger().error("Failed to communicate")

def main(args=None):
    rclpy.init(args=args)

    node = NerfFirer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.change_motor_state(False)
        node.change_trigger_state(False)
        node.cancel_trigger_timer()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()