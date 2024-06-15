#!/usr/bin/env python3

import math
import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# Media pip libs
import cv2
import mediapipe as mp
from cv_bridge import CvBridge

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

BUFFER_SIZE = 5
DEPTH_SCALE = 0.001

class ArHandTracker(Node):
    def __init__(self, tracker):
        super().__init__("ar_hand_tracker")

        
        self.tracker = tracker
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.cv_bridge = CvBridge()

        self.tf_publisher = self.create_publisher(TransformStamped, "/ar_hand_tracker/tf", 10)

        self.x_buffer = []
        self.y_buffer = []
        self.z_buffer = []
        self.detection = None
        self._point_cloud = None
        self._color_img = None
        self._depth_img = None
        

        # self.point_cloud_sub = self.create_subscription(
        #     PointCloud2,
        #     "/camera/depth/color/points",
        #     self.point_cloud_callback, 10
        # ) 

        self.image_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback, 10
        )

        self.depth_image_sub = self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.depth_image_callback, 10
        )
    
    def point_cloud_callback(self, msg: PointCloud2):
        #print(f"H: {msg.height}, W: {msg.width}")

        # mid_x = int(1280 / 2)
        # mid_y = int(720 / 2)

        # # pos = mid_x * mid_y
        # # el = msg.data
        # # print(el)

        # res = point_cloud2.read_points(msg, uvs=(mid_x, mid_y))
        # print(res)
        self._point_cloud = msg

    def image_callback(self, image):
        self._color_img = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        self.run_detection()

    def depth_image_callback(self, msg: Image):
        self._depth_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.run_detection()

    def run_detection(self):
        if self._color_img is None \
            or self._depth_img is None:
            self.get_logger().info("NO DATA YET")
            return
        
        image = cv2.cvtColor(self._color_img, cv2.COLOR_BGR2RGB)
        image_height, image_width, _ = self._color_img.shape
        results = self.tracker.process(image)

        # Draw the face detection annotations on the image.
        image.flags.writeable = False
        
        if results.multi_hand_landmarks:
            # Only track one hand
            
            hand_landmarks = results.multi_hand_landmarks[0]
            x = [landmark.x for landmark in hand_landmarks.landmark]
            y = [landmark.y for landmark in hand_landmarks.landmark]
            
            center = np.array([np.mean(x)*image_width, np.mean(y)*image_height]).astype('int32')
            
            cv2.circle(image, tuple(center), 10, (255,0,0), 1)  #for checking the center 
            cv2.rectangle(image, (center[0]-200,center[1]-200), (center[0]+200,center[1]+200), (255,0,0), 1)
            
            mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style())
            
            
            if center[0] <= image_width and center[1] <= image_height:
                self.handle_detection(center[0], center[1])

            cv2.imshow('MediaPipe Hand Detection', image)
            if cv2.waitKey(5) & 0xFF == 27:
                return


    def handle_detection(self, pixel_x, pixel_y):
        
        image_height, image_width, _ = self._color_img.shape

        depth_px = self._depth_img[int(pixel_y)][int(pixel_x)]
        
        depth = depth_px * DEPTH_SCALE

        # FOV of camera: 65° × 40°
        # Dimension at the distance from camera:
        # b = a × tan(β)
        
        width_x = (depth * math.tan( math.radians(65 / 2))) * 2
        width_y = (depth * math.tan( math.radians(40 / 2))) * 2
        # Find the position of the 

        point_x = ((pixel_x / image_width) - 0.5) * width_x
        point_y = ((pixel_y / image_height) - 0.5) * width_y
        point_z = depth

        try:

            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_color_optical_frame'
            t.child_frame_id = "tracked_hand"

            # self.get_logger().info(f"type: {type(x)}, {type(y)} -> {type(z)}")

            t.transform.translation.x = point_x
            t.transform.translation.y = point_y
            t.transform.translation.z = point_z
            # quat = self.quaternion_from_euler(
            #     float(transformation[5]), float(transformation[6]), float(transformation[7]))
            # t.transform.rotation.x = quat[0]
            # t.transform.rotation.y = quat[1]
            # t.transform.rotation.z = quat[2]
            # t.transform.rotation.w = quat[3]

            self.tf_static_broadcaster.sendTransform(t)
            self.tf_publisher.publish(t)
        except Exception as ex:
            self.get_logger().error(ex)


    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

def main_old():
    rclpy.init()
    node = ArHandTracker(None)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

def main():
    rclpy.init()
    with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as tracker:
        node = ArHandTracker(tracker)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()
