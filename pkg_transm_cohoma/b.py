#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from visualization_msgs.msg import Marker
import numpy as np

class YOLOv8Detector3D:
    def __init__(self):
        rospy.init_node('yolov8_3d_detector', anonymous=True)
        self.bridge = CvBridge()

        # YOLO model
        model_path = rospy.get_param('~model_path', 'yolov8n.pt')
        self.model = YOLO(model_path)

        # Camera intrinsics
        self.K = None  # Will be filled from /camera_info

        # Image placeholders
        self.depth_image = None

        # Subscribers
        self.image_sub = rospy.Subscriber('/robot/front_rgbd_camera/color/image_raw', Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.depth_sub = rospy.Subscriber('/robot/front_rgbd_camera/depth/image_raw', Image, self.depth_callback, queue_size=1, buff_size=2**24)
        self.cam_info_sub = rospy.Subscriber('/robot/front_rgbd_camera/color/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)

        # Publishers
        self.image_pub = rospy.Publisher('/yolo/image_annotated', Image, queue_size=1)
        self.marker_pub = rospy.Publisher('/yolo/markers', Marker, queue_size=10)
        self.marker_id = 0  # Pour les IDs uniques de markers


        rospy.loginfo(f"[YOLOv8-3D] Node started with model: {model_path}")

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.K).reshape(3, 3)
            rospy.loginfo("[YOLOv8-3D] Camera intrinsics received.")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(f"[YOLOv8-3D] Depth image conversion failed: {e}")

    def image_callback(self, msg):
        if self.K is None or self.depth_image is None:
            rospy.logwarn("[YOLOv8-3D] Waiting for depth and camera intrinsics...")
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"[YOLOv8-3D] RGB image conversion failed: {e}")
            return

        results = self.model.predict(color_image, verbose=False)
        annotated_img = results[0].plot()

        for det in results[0].boxes.data:
            x1, y1, x2, y2, conf, cls = map(int, det[:6])

            # Crop the depth image
            depth_crop = self.depth_image[y1:y2, x1:x2]

            points_3d = []
            for v in range(y1, y2):
               for u in range(x1, x2):
                   Z = self.depth_image[v, u]
                   if Z == 0 or np.isnan(Z):
                      continue
                   Z = Z / 1000.0
                   X = (u - self.K[0, 2]) * Z / self.K[0, 0]
                   Y = (v - self.K[1, 2]) * Z / self.K[1, 1]
                   points_3d.append([X, Y, Z])

            if len(points_3d) < 10:
               rospy.logwarn("[YOLOv8-3D] Not enough valid depth points for 3D bbox.")
               continue

            points_np = np.array(points_3d)
            min_pt = points_np.min(axis=0)
            max_pt = points_np.max(axis=0)

            rospy.loginfo(f"[YOLOv8-3D] 3D BBox [{int(cls)}] → min: {min_pt}, max: {max_pt}")

            try:
                marker = self.create_bbox_marker(min_pt, max_pt, int(cls), msg.header)
                self.marker_pub.publish(marker)
            except Exception as e:
                rospy.logerr(f"[YOLOv8-3D] Marker publish failed: {e}")

            
    def create_bbox_marker(self, min_pt, max_pt, cls_id, header):
        marker = Marker()
        marker.header = header
        marker.ns = "yolo3d"
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position = centre du cube
        center = (min_pt + max_pt) / 2.0
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]

        # Pas de rotation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Dimensions
        size = max_pt - min_pt
        size[size < 0.01] = 0.01
        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]

        # Couleur 
        color = self.class_color(cls_id)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.5  # transparence

        marker.lifetime = rospy.Duration(0.2)  # effacer si non mis à jour

        return marker

    def class_color(self, cls_id):
        # Palette simple 
        colors = [
            (1.0, 0.0, 0.0),  # rouge
            (0.0, 1.0, 0.0),  # vert
            (0.0, 0.0, 1.0),  # bleu
            (1.0, 1.0, 0.0),  # jaune
            (1.0, 0.0, 1.0),  # magenta
            (0.0, 1.0, 1.0),  # cyan
        ]  
        return colors[cls_id % len(colors)]


if __name__ == '__main__':
    try:
        YOLOv8Detector3D()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

