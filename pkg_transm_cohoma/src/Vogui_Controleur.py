#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import Commande_vogui as cm
from std_msgs.msg import String, ByteMultiArray

class Vogui_Controleur:
    def __init__(self):
        rospy.init_node('vogui_controleur_node', anonymous=True)
        rospy.Subscriber('/robot/topicCmd', ByteMultiArray, self.cb_cmd)
        #rospy.loginfo("")
        self.RUNNING1 = False
        self.RUNNING2 = False
        self.STREAM_ON1 = False
        self.DETECT_ON1 = False
        self.STREAM_ON2 = False
        self.DETECT_ON2 = False
        self.EXIT1 = False
        self.EXIT2 = False
    
   
        
    def cb_cmd(self,msg):
        cmd=cm.Commande(self)
        cmd.data=msg.data
        type=cmd.getTypeCommande()
        cmd.GetParamStream()


        #self.get_logger().info(f"La caméra 1 marche : {self.RUNNING1} et la caméra 2 marche : {self.RUNNING2}")
        if(not(self.RUNNING1) and cmd.NUMCAM == 0):
            self.RUNNING1 = True 
            self.EXIT1 = False
            cmd.start()             
        if(not(self.RUNNING2) and cmd.NUMCAM == 2):
            self.RUNNING2 = True
            self.EXIT2 = False
            cmd.start2()

        if type==cm.CMD_STARTSTREAM:
            # (REMOTE_IP,REMOTE_PORT,numcam)=cmd.GetParamStream()
            # self.process_stream[numcam]=cmd.StartStream()
            # if self.process_stream[numcam]==0:
            #     print('stream lancé')

            if(cmd.NUMCAM == 0):
                self.STREAM_ON1 = True
            elif(cmd.NUMCAM == 2):
                self.STREAM_ON2 = True


        if type==cm.CMD_STOPSTREAM:
            #(REMOTE_IP,REMOTE_PORT,numcam)=cmd.GetParamStream()
            #self.process_stream[numcam].terminate()
            if(cmd.NUMCAM == 0):
                self.STREAM_ON1 = False
            elif(cmd.NUMCAM == 2):
                self.STREAM_ON2 = False
        

        #self.get_logger().info(f"STREAM1 : {self.STREAM_ON1} , STREAM2 : {self.STREAM_ON2} , DETECT1 : {self.DETECT_ON1} , DETECT2 : {self.DETECT_ON2}, CAM1 : {self.RUNNING1}, CAM2 : {self.RUNNING2}")
        if(self.RUNNING1 and not(self.DETECT_ON1 or self.STREAM_ON1)):
            cmd.stop()
            self.RUNNING1 = False


        if(self.RUNNING2 and not(self.DETECT_ON2 or self.STREAM_ON2)):
            cmd.stop2()
            self.RUNNING2 = False


    def start(self):
        rospy.spin()
        
if __name__ == "__main__":
    try:
        node = Vogui_Controleur()
        node.start()
    except rospy.ROSInterruptException:
        pass

