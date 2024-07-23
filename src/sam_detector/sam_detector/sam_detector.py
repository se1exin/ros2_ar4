import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ultralytics import FastSAM
from ultralytics.models.fastsam import FastSAMPrompt
import numpy as np
from scipy.spatial.transform import Rotation

DEPTH_SCALE = 0.001
CAM_FOV_X = 70.0  # Docs say 65deg
CAM_FOV_Y = 45.0  # Docs say 40deg

class SAMDetector(Node):
    def __init__(self):
        super().__init__("sam_detector")
        self.logger = self.get_logger()

        # Initialize a YOLO-World model
        self.object_model = YOLO("yolov8l-world.pt")
        self.object_model.set_classes(["marker", "bowl", "pen", "circle"])
        
        #self.segment_model = YOLO("yolov8n-seg.pt")
        # Create a FastSAM model
        self.segment_model = FastSAM("FastSAM-s.pt")  # or FastSAM-x.pt
        
        self.cv_bridge = CvBridge()

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_publisher = self.create_publisher(TransformStamped, "/sam_detector/tf", 10)
        
        self._color_img = None
        self._depth_img = None
        self.bowl_bbox = None

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
        
        image = cv2.cvtColor(self._color_img, cv2.COLOR_BGR2RGB)

        # Detect bounding boxes of desired objects in the image
        object_detection = self.object_model.predict(image)

        # Extract the bounding box for pen (marker) if present
        pen_bbox = self.get_object_bbox(object_detection, "pen")

        # Extract the bounding box of the bowl (if present)
        bowl_bbox = self.get_object_bbox(object_detection, "bowl")

        

        if pen_bbox is None:
            self.logger.warn("No pen found!")
            # Send a blank transform so the arm doesn't keep trying at the last known location
            self.publish_transform(Transform(), frame_id="gripper_target_object")
            return
        
        if bowl_bbox is None and self.bowl_bbox is None:
            self.logger.warn("No bowl found!")
            return
        
        # Cache the bowl, as it is hard to track, but rarely moves
        if bowl_bbox is None:
            bowl_bbox = self.bowl_bbox
        else:
            self.bowl_bbox = bowl_bbox
 
        contour = self.detect_contour(image, pen_bbox)
        if contour is None:
            self.logger.warn("Could not detect mask of pen")
            return

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

        camera_rot = Rotation.from_euler("y", 27.0, degrees=True)  # Account for angle of camera
        extra_rot = Rotation.from_euler("z", gripper_rotation, degrees=True)  # Rotation of object
        grasp_quat = (camera_rot * extra_rot * top_down_rot).as_quat()
        
        object_transform.translation.x = pen_x
        object_transform.translation.y = pen_y
        object_transform.translation.z = pen_z
        object_transform.rotation.x = grasp_quat[0]
        object_transform.rotation.y = grasp_quat[1]
        object_transform.rotation.z = grasp_quat[2]
        object_transform.rotation.w = grasp_quat[3]
        self.publish_transform(object_transform, frame_id="gripper_target_object")

        ## BOWL POSITION
        bowl_x, bowl_y, bowl_z = self.translate_xy_to_xyz(bowl_bbox["center_x"], bowl_bbox["center_y"])

        bowl_transform = Transform()
        top_down_rot = Rotation.from_quat([
            bowl_transform.rotation.x,
            bowl_transform.rotation.y,
            bowl_transform.rotation.z,
            bowl_transform.rotation.w,
        ])
        extra_rot = Rotation.from_euler("z", 90.0, degrees=True)  # Rotation of object
        drop_quat = (camera_rot * extra_rot * top_down_rot).as_quat()
        bowl_transform.translation.x = bowl_x
        bowl_transform.translation.y = bowl_y
        bowl_transform.translation.z = bowl_z
        bowl_transform.rotation.x = drop_quat[0]
        bowl_transform.rotation.y = drop_quat[1]
        bowl_transform.rotation.z = drop_quat[2]
        bowl_transform.rotation.w = drop_quat[3]
        self.publish_transform(bowl_transform, frame_id="detected_bowl")



        contour = contour.reshape(-1, 1, 2)
        _ = cv2.drawContours(image, [contour], -1, (255, 255, 255), cv2.FILLED)         
        cv2.rectangle(image, (pen_bbox["x1"], pen_bbox["y1"]), (pen_bbox["x2"], pen_bbox["y2"]), (255,0,0), 1)
        cv2.rectangle(image, (bowl_bbox["x1"], bowl_bbox["y1"]), (bowl_bbox["x2"], bowl_bbox["y2"]), (255,0,0), 1)
        cv2.imshow('sam_detector', image)
        if cv2.waitKey(5) & 0xFF == 27:
            return



    def get_object_bbox(self, detection, class_name=""):
        if len(detection) == 0:
            return None

        for idx, box in enumerate(detection[0].boxes.xywh):
            clsIdx = detection[0].boxes.cls.tolist()[idx]
             
            label = detection[0].names[clsIdx]
            if label != class_name:
                continue

            bbox = box.detach().cpu().numpy()
            center_x = int(bbox[0])
            center_y = int(bbox[1])
            width_x = int(bbox[2])
            width_y = int(bbox[3])
            start_x = int(center_x - (width_x / 2))
            start_y = int(center_y - (width_y / 2))
            end_x = start_x + width_x
            end_y = start_y + width_y
            return {
                "x1": start_x,
                "y1": start_y,
                "x2": end_x,
                "y2": end_y,
                "center_x": center_x,
                "center_y": center_y
            }

        return None
    
    def detect_contour(self, image, bbox):
        # Run segment inference on an image
        everything_results = self.segment_model(image, device="cuda", retina_masks=True, imgsz=1024, conf=0.4, iou=0.9)
        # Prepare a Prompt Process object
        prompt_process = FastSAMPrompt(image, everything_results, device="cuda")


        # Bbox default shape [0,0,0,0] -> [x1,y1,x2,y2]
        result = prompt_process.box_prompt(bbox=[bbox["x1"], bbox["y1"], bbox["x2"], bbox["y2"]])

        if len(result) > 0:
            masks = result[0].masks

            #  Extract contour result
            contour = masks.xy.pop()
            #  Changing the type
            contour = contour.astype(np.int32)
            return contour

        return None
    

    def contour_to_gripper_rotation(self, contour):
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

    def publish_transform(self, transform: Transform, frame_id = ""):
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_color_optical_frame'
            t.child_frame_id = frame_id
            t.transform = transform
            self.tf_static_broadcaster.sendTransform(t)
            self.tf_publisher.publish(t)
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