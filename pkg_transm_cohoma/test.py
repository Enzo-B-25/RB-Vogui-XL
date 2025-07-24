#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

image_received = False

def callback(msg):
    global image_received
    if len(msg.data) > 0:
        image_received = True
        rospy.signal_shutdown("Image received")

rospy.init_node("wait_for_image")
rospy.Subscriber("/robot/front_rgbd_camera/color/image_raw", Image, callback)

rospy.loginfo("Waiting for image data on /robot/front_rgbd_camera/color/image_raw...")
rospy.spin()

# Quand image reçue, lancer rtabmap
import subprocess
subprocess.call(["roslaunch", "pkg_transm_cohoma", "rtabmap_vogui.launch"])

