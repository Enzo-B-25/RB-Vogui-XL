#!/usr/bin/env python3

import rospy
import serial
import struct
from std_msgs.msg import String
from sensor_msgs.msg import Joy, NavSatFix
from robotnik_msgs.msg import BatteryStatus
import SerialFrame as SF


class SerialNode:
    def __init__(self):
        self.processor = SF.SerialFrame(port='/dev/ttyUSB1', baudrate=57600)
        self.data_to_send = SF.TOSEND_NO
        self.Joy = Joy()
        self.Joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Joy.buttons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        self.Batt = BatteryStatus()
        self.GPS = NavSatFix()
        # Publisher pour envoyer les données lues
        self.pub_joy = rospy.Publisher('/robot/joyRecu', Joy, queue_size=10)

        self.sub_gps = rospy.Subscriber("/robot/gps/fix", NavSatFix, self.cb_gps)
        self.sub_batt = rospy.Subscriber("/robot/battery_estimator/data", BatteryStatus, self.cb_batt)
        self.sub_batt = rospy.Subscriber("/robot/battery_estimator/data", BatteryStatus, self.cb_IMU)

        # Timer pour lire les trames périodiquement
        rospy.Timer(rospy.Duration(0.5), self.read_serial_data)

        # Timer pour envoyer des trames périodiquement (toutes les secondes)
        rospy.Timer(rospy.Duration(0.5), self.send_serial_data)
        rospy.loginfo("init relais DR OK")

    def read_serial_data(self, event):
        """Fonction appelée périodiquement pour lire les trames série."""
        frame = self.processor.read_frame()
        if frame:
            rospy.loginfo("Trame reçue")
            if frame[0] == SF.TYPE_JOY:
                msg = self.processor.decode_joy(frame)
                self.pub_joy.publish(msg)
                rospy.loginfo("Trame joy")
                rospy.loginfo(msg)


    def send_serial_data(self, event):
        """Fonction appelée périodiquement pour envoyer des trames série."""
        #rospy.loginfo(f"Acces fonction envoyer")
        if self.data_to_send == SF.TOSEND_TELEM:
            #rospy.loginfo(f"Envoie d'une trame")
            self.processor.send_frame(self.processor.encode_telemetry(self.GPS, self.Batt))
            #rospy.loginfo(f"Trame envoyée: {self.data_to_send}")
        
        self.data_to_send = SF.TOSEND_NO
    
    def cb_gps(self, msg):
        # rospy.loginfo("cb_gps")
        self.GPS = msg
        self.data_to_send = SF.TOSEND_TELEM

    def cb_batt(self, msg):
        self.Batt = msg


def main():
    rospy.init_node('serial_node')
    serial_node = SerialNode()
    rospy.spin()

if __name__ == '__main__':
    main()
