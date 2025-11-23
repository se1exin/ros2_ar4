import math

import requests
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform, TransformStamped
from tf2_ros import TransformBroadcaster

from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

DEPTH_SCALE = 0.001
CAM_FOV_X = 70.0  # Docs say 65deg
CAM_FOV_Y = 45.0  # Docs say 40deg
CAMERA_ANGLE = 55.0  # Camera is 55
SERVER_URL = "http://10.1.1.8:8000/predict/"

class SAMDetector(Node):
    def __init__(self):
        super().__init__("hand_object_detector")
        self.logger = self.get_logger()
        
        self.cv_bridge = CvBridge()

        self.tf_broadcaster = TransformBroadcaster(self)
        
        self._color_img = None
        self._depth_img = None
        self.img_width = 0
        self.img_height = 0

        self.image_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback, 10
        )

        self.depth_image_sub = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_image_callback, 10
        )

        self.processed_image_pub = self.create_publisher(Image, "/sam_detector/image", 1)

        self.is_running = False
        self.last_run_time = None
        self.detection_interval = rclpy.duration.Duration(seconds=1.0)

        self.transform = Transform()
        # self.publish_transform_timer = self.create_timer(0.1, self.publish_transform)
    
    def image_callback(self, image):
        self._color_img = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        self.run_detection()
    
    def depth_image_callback(self, msg: Image):
        self._depth_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.run_detection()

    def run_detection(self):
        if self._color_img is None \
            or self._depth_img is None:
            self.get_logger().info("Waiting for RGB and Depth data...")
            return

        if self.is_running:
            self.get_logger().info("Detection is already running...")
            return
        
                
        now = self.get_clock().now()

        # Only run detection if 3 seconds have passed
        if self.last_run_time is not None and (now - self.last_run_time) < self.detection_interval:
            return  # Skip detection

        self.last_run_time = now
        self.is_running = True

        # Crop to square center
        h, w, _ = self._color_img.shape
        min_dim = min(h, w)
        start_x = (w - min_dim) // 2
        start_y = (h - min_dim) // 2
        self.img_width = min_dim
        self.img_height = min_dim
        cropped_img = self._color_img[start_y:start_y+min_dim, start_x:start_x+min_dim]
        image = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2RGB)
        
        
        # self.img_width = w
        # self.img_height = h
        # image = cv2.cvtColor(self._color_img, cv2.COLOR_BGR2RGB)

        # contour = self.detect_contour(image, pen_bbox)
        contour, hand_bbox, object_bbox = self.detect_contour_remote(image)
        if contour is None or hand_bbox is None or object_bbox is None:
            # Send a transform with zero translation to indicate no object
            # self.transform = Transform()
            # self.publish_transform()
            self.is_running = False
            return
        
        # The object bbox is relative to the pen bbox
        pen_bbox = {
            "x1": object_bbox["x1"] + hand_bbox["x1"],
            "y1": object_bbox["y1"] + hand_bbox["y1"],
            "x2": object_bbox["x2"] + hand_bbox["x1"],
            "y2": object_bbox["y2"] + hand_bbox["y1"],
        }
        pen_bbox["center_x"] = (pen_bbox["x1"] + pen_bbox["x2"]) / 2
        pen_bbox["center_y"] = (pen_bbox["y1"] + pen_bbox["y2"]) / 2

        # The contour is relative to the hand bbox
        if contour.ndim == 3 and contour.shape[1] == 1:
            contour[:, 0, 0] += hand_bbox["x1"]
            contour[:, 0, 1] += hand_bbox["y1"]
        else:
            contour[:, 0] += hand_bbox["x1"]
            contour[:, 1] += hand_bbox["y1"]


        contour = contour.reshape(-1, 1, 2)
        _ = cv2.drawContours(image, [contour], -1, (255, 255, 255), cv2.FILLED)         
        cv2.rectangle(image, (pen_bbox["x1"], pen_bbox["y1"]), (pen_bbox["x2"], pen_bbox["y2"]), (255,0,0), 3)
        # cv2.imshow("SAM Detection", image)
        # cv2.waitKey(1)

        # Convert BGR image back from RGB before publishing
        image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.processed_image_pub.publish(image_msg)

        

        ## PEN POSITION
        # Translate center of box of the pen to the real-world position relative to camera frame
        pen_x, pen_y, pen_z = self.translate_xy_to_xyz(pen_bbox["center_x"], pen_bbox["center_y"])
    
        # Get the rotation of the object from the contour of it's mask
        gripper_rotation = self.contour_to_gripper_rotation(contour)
        
        object_transform = Transform()
        # Calculate rotation
        # Based on https://github.com/ycheng517/tabletop-handybot/blob/6d401e577e41ea86529d091b406fbfc936f37a8d/tabletop_handybot/tabletop_handybot/tabletop_handybot_node.py#L394
        top_down_rot = Rotation.from_quat([
            object_transform.rotation.x,
            object_transform.rotation.y,
            object_transform.rotation.z,
            object_transform.rotation.w,
        ])

        camera_rot = Rotation.from_euler("x", CAMERA_ANGLE, degrees=True)  # Account for angle of camera
        extra_rot = Rotation.from_euler("z", gripper_rotation, degrees=True)  # Rotation of object
        # grasp_quat = (camera_rot * extra_rot * top_down_rot).as_quat()
        
        # Rotate 90 around the y axis to account for gripper orientation
        x_rot = Rotation.from_euler("x", 0, degrees=True)
        y_rot = Rotation.from_euler("y", 90, degrees=True)
        z_rot = Rotation.from_euler("z", CAMERA_ANGLE, degrees=True)
        grasp_quat = x_rot * y_rot * z_rot
        grasp_quat = grasp_quat.as_quat()
        
        object_transform.translation.x = pen_x
        object_transform.translation.y = pen_y
        object_transform.translation.z = pen_z
        object_transform.rotation.x = grasp_quat[0]
        object_transform.rotation.y = grasp_quat[1]
        object_transform.rotation.z = grasp_quat[2]
        object_transform.rotation.w = grasp_quat[3]
        self.transform = object_transform
        self.publish_transform()

        self.is_running = False

    def detect_contour_remote(self, image):

        # Convert OpenCV image (RGB) to JPEG bytes
        _, img_encoded = cv2.imencode('.jpg', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        img_bytes = img_encoded.tobytes()

        try:
            files = {
                'file': ('image.jpg', img_bytes, 'image/jpeg')
            }
            response = requests.post(SERVER_URL, files=files, timeout=10)
            if response.status_code == 200:
                result = response.json()
                if result["contour"] is None or result["hand_bbox"] is None or result["object_bbox"] is None:
                    self.logger.error("Remote FastSAM did not detect any contour or bbox")
                    return None, None, None
                
                contour = np.array(result["contour"], dtype=np.int32)
                hand_bbox = result["hand_bbox"]
                object_bbox = result["object_bbox"]
                # self.logger.info(f"Remote FastSAM response: {contour}, {bbox}")
                return contour, hand_bbox, object_bbox
            else:
                self.logger.error(f"Remote FastSAM error: {response.status_code}, {response.text}")
                return None, None, None

        except Exception as e:
            self.logger.error(f"Error connecting to FastSAM server: {e}")
            return None, None, None

    def contour_to_gripper_rotation(self, contour):
        # return 0
        # Contour is the contour of the detected mask
        center, dimensions, theta = cv2.minAreaRect(contour)
        gripper_rotation = theta
        if dimensions[0] > dimensions[1]:
            gripper_rotation -= 90
        if gripper_rotation < -90:
            gripper_rotation += 180
        elif gripper_rotation > 90:
            gripper_rotation -= 180
        
        return gripper_rotation
            

    # Given an x,y coordinate of the camera image, convert it to
    # an x,y,z in the camera frame given the camera FOV and depth scale
    def translate_xy_to_xyz(self, pixel_x, pixel_y):
        image_height, image_width, _ = self._color_img.shape
        
        img_width_diff = (image_width - self.img_width) / 2
        img_height_diff = (image_height - self.img_height) / 2

        pixel_x += img_width_diff
        pixel_y += img_height_diff

        depth_px = self._depth_img[int(pixel_y)][int(pixel_x)]
        
        depth = depth_px * DEPTH_SCALE

        # FOV of camera: 65° × 40°
        # Dimension at the distance from camera:
        # b = a × tan(β)
        
        width_x = (depth * math.tan( math.radians(CAM_FOV_X / 2))) * 2
        width_y = (depth * math.tan( math.radians(CAM_FOV_Y / 2))) * 2
        # Find the position of the 

        point_x = ((pixel_x / image_width) - 0.5) * width_x
        point_y = ((pixel_y / image_height) - 0.5) * width_y
        point_z = depth

        return point_x, point_y, point_z

    def publish_transform(self):
        try:

            tf = TransformStamped()
            tf.header.frame_id = 'camera_color_optical_frame'
            tf.child_frame_id = 'gripper_target_object'
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.transform = self.transform
            self.tf_broadcaster.sendTransform(tf)
        except Exception as ex:
            self.get_logger().error(ex)

def main():
    rclpy.init()
    node = SAMDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()