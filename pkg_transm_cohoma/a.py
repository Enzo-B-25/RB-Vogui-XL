#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOv8Detector:
    def __init__(self):
        rospy.init_node('yolov8_detector', anonymous=True)

        # Bridge ROS ↔ OpenCV
        self.bridge = CvBridge()

        # Charger le modèle YOLOv8 (par défaut : yolov8n.pt ou yolov8s.pt)
        model_path = rospy.get_param('~model_path', 'yolov8n.pt')
        self.model = YOLO(model_path)

        # Souscription à l'image RGB
        self.image_sub = rospy.Subscriber(
            '/robot/front_rgbd_camera/color/image_raw',
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        # Publication de l'image annotée
        self.image_pub = rospy.Publisher(
            '/yolo/image_annotated',
            Image,
            queue_size=1
        )

        rospy.loginfo(f"[YOLOv8] Node started with model: {model_path}")

    def image_callback(self, msg):
        try:
            # Conversion de ROS → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"[YOLOv8] Erreur conversion image: {e}")
            return

        # Inference YOLOv8
        results = self.model.predict(cv_image, verbose=False)

        # Annoter l'image
        annotated_img = results[0].plot()

        # Conversion OpenCV → ROS + publication
        try:
            ros_image = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
            ros_image.header = msg.header  # garder les infos temporelles
            self.image_pub.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"[YOLOv8] Erreur publication image: {e}")

if __name__ == '__main__':
    try:
        YOLOv8Detector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

