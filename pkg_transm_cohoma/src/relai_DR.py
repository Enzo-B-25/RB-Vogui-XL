#!/usr/bin/env python3

import rospy
import serial
import struct
from std_msgs.msg import String, ByteMultiArray
from sensor_msgs.msg import Joy, NavSatFix, Imu
from robotnik_msgs.msg import BatteryStatus
import SerialFrame as SF


class SerialNode:
    def __init__(self):
        # Initialisation de la liaison série avec la radio
        self.processor = SF.SerialFrame(port='/dev/DR_DT2', baudrate=57600)
        self.data_to_send = SF.TOSEND_NO

        # Messages initiaux
        self.Joy = Joy()
        self.Joy.axes = [0.0] * 6
        self.Joy.buttons = [0] * 16
        self.Batt = BatteryStatus()
        self.GPS = NavSatFix()
        self.IMU = Imu()

        # Joy neutre pour arrêt sécurité
        self.stop_joy = Joy()
        self.stop_joy.axes = [0.0] * 6
        self.stop_joy.buttons = [0] * 16
        self.joy_timed_out = False  # Évite de republier en boucle

        # Publishers
        self.pub_joy = rospy.Publisher('/robot/joyRecu', Joy, queue_size=10)
        self.pub_cmd = rospy.Publisher('/robot/topicCmd', ByteMultiArray, queue_size=10)

        # Timers ROS
        rospy.Timer(rospy.Duration(0.2), self.read_serial_data)      # Lecture trame série toutes les 200 ms
        rospy.Timer(rospy.Duration(0.2), self.send_serial_data)      # Envoi trame série toutes les 200 ms
        rospy.Timer(rospy.Duration(0.2), self.check_joy_timeout)     # Sécurité : vérifie toutes les 100 ms

        # Timeout de sécurité pour l'absence de Joy
        self.last_joy_time = rospy.Time.now()
        self.cmd_vel_timeout = rospy.Duration(0.2)  # 250 ms de tolérance

        # Souscriptions capteurs
        self.sub_gps = rospy.Subscriber("/fix", NavSatFix, self.cb_gps)
        self.sub_batt = rospy.Subscriber("/robot/battery_estimator/data", BatteryStatus, self.cb_batt)
        self.sub_imu = rospy.Subscriber("/robot/imu/data", Imu, self.cb_imu)

        rospy.loginfo("init relais DR OK")

    def read_serial_data(self, event):
        # Lecture périodique des trames série entrantes
        frame = self.processor.read_frame()
        if frame:
            rospy.loginfo("Trame reçue")
            if frame[0] == SF.TYPE_JOY:
                msg = self.processor.decode_joy(frame)
                self.pub_joy.publish(msg)
                self.last_joy_time = rospy.Time.now()  # Maj du timestamp
                self.joy_timed_out = False              # Réinitialise le flag
                rospy.loginfo("Trame joy")
                rospy.loginfo(msg)

            if frame[0] == SF.TYPE_CMD:
                msg = self.processor.decode_cmd(frame)
                self.pub_cmd.publish(msg)
                rospy.loginfo("long frame " + str(frame))
                rospy.loginfo(msg)
                rospy.loginfo("Trame CMD")

    def check_joy_timeout(self, event):
        # Si plus de 250 ms sans trame joy, publie Joy nul
        if rospy.Time.now() - self.last_joy_time > self.cmd_vel_timeout:
            if not self.joy_timed_out:
                self.pub_joy.publish(self.stop_joy)
                rospy.logwarn("Timeout joy → publication d'un Joy nul sur /robot/joyRecu.")
                self.joy_timed_out = True

    def send_serial_data(self, event):
        # Envoi périodique des trames série (télémétrie ou pose)
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

