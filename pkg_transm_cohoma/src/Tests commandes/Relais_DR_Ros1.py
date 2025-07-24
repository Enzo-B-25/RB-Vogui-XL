#!/usr/bin/env python3

import rospy
import serial
import struct
from std_msgs.msg import String
from sensor_msgs.msg import Joy, NavSatFix, Imu
from robotnik_msgs.msg import BatteryStatus
import SerialFrame as SF


class SerialNode:
    def __init__(self):
        # Récupération du port série depuis les paramètres (par défaut : /dev/ttyUSB0)
        device = rospy.get_param('~device', '/dev/ttyUSB0')
        self.processor = SF.SerialFrame(port=device, baudrate=57600)

        self.data_to_send = SF.TOSEND_NO

        self.Joy = Joy()
        self.Joy.axes =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.Batt = BatteryStatus()
        self.GPS = NavSatFix()
        self.IMU = Imu()

        # Publisher pour envoyer les données Joy reçues
        self.pub_joy = rospy.Publisher('/robot/joyRecu', Joy, queue_size=10)

        # Souscriptions aux capteurs
        self.sub_gps = rospy.Subscriber("/robot/gps/fix", NavSatFix, self.cb_gps)
        self.sub_batt = rospy.Subscriber("/robot/battery_estimator/data", BatteryStatus, self.cb_batt)
        self.sub_imu = rospy.Subscriber("/robot/imu/data", Imu, self.cb_imu)

        # Timer pour lire les trames série régulièrement
        rospy.Timer(rospy.Duration(0.5), self.read_serial_data)

        # Timer pour envoyer les trames
        rospy.Timer(rospy.Duration(0.5), self.send_serial_data)

        rospy.loginfo("init relais DR OK")

    def read_serial_data(self, event):
        #Lecture périodique des trames série entrantes
        frame = self.processor.read_frame()
        if frame:
            rospy.loginfo("Trame reçue")
            if frame[0] == SF.TYPE_JOY:
                msg = self.processor.decode_joy(frame)
                self.pub_joy.publish(msg)
                rospy.loginfo("Trame joy")
                rospy.loginfo(msg)

    def send_serial_data(self, event):
        #Envoi périodique des trames série
        if self.data_to_send == SF.TOSEND_TELEM:
            self.processor.send_frame(self.processor.encode_telemetry(self.GPS, self.Batt, self.IMU))
        elif self.data_to_send == SF.TOSEND_POSE:
            self.processor.send_frame(self.processor.encode_pose(self.Pose))
        
        self.data_to_send = SF.TOSEND_NO

    def cb_gps(self, msg):
        self.GPS = msg
        self.data_to_send = SF.TOSEND_TELEM

    def cb_batt(self, msg):
        self.Batt = msg

    def cb_imu(self, msg):
        self.IMU = msg


def main():
    rospy.init_node('serial_node')
    serial_node = SerialNode()
    rospy.spin()

if __name__ == '__main__':
    main()