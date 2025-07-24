import cv2
import numpy as np
import sys
import struct
import threading
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import  String
import rospy

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_RATE = 10
CONFIDENCE_THRESHOLD = 0.5
FRAME_SKIP = 10

CMD_NOCMD=0
CMD_STARTSTREAM=1
CMD_STOPSTREAM=2
CMD_START_DD=3
CMD_STOP_DD=4

class Commande:
    def __init__(self, node):
        # Variables d'état
        self.IP = ""
        self.PORT = -1
        self.NUMCAM = -1

        self.node = node
        self.type=0
        self.data=''


        # Modèle YOLO chargé une fois
        #self.model = YOLO("/root/ros_ws/src/battle-center/battle_center/best.pt")

        # Threads et ressources caméra/stream
        self.thread = None
        self.cap = None
        self.out = None
        self.thread2 = None
        self.cap2 = None
        self.out2 = None




    def getTypeCommande(self):
        if len(self.data)>0:
            # print(type(self.data[0]))
            # return int(self.data[0])
            #return int.from_bytes(self.data[0], byteorder='big')
            return int(self.data[0])
        else:
            return 0

    def GetParamStream(self):
        #(a,b,c,d,port,numcam)=struct.unpack('BBBBHB',b''.join((self.data[1:])))
        raw = [(x + 256) if x < 0 else x for x in self.data[1:]]
        (a, b, c, d, port, numcam) = struct.unpack('BBBBHB', bytes(raw))


        self.IP = str(a)+'.'+str(b)+'.'+str(c)+'.'+str(d)
        self.PORT = port
        self.NUMCAM = numcam


    def open_camera(self):
        self.cap = cv2.VideoCapture(self.NUMCAM, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)

        if not self.cap.isOpened():
            rospy.loginfo(f"Erreur1 : Impossible d'ouvrir la caméra {self.NUMCAM}")
            sys.exit(1)
        rospy.loginfo(f"✅ 1Caméra {self.NUMCAM} ouverte")



    def open_camera2(self):
        self.cap2 = cv2.VideoCapture(self.NUMCAM, cv2.CAP_V4L2)
        self.cap2.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap2.set(cv2.CAP_PROP_FPS, FRAME_RATE)

        if not self.cap2.isOpened():
            rospy.loginfo(f"Erreur2 : Impossible d'ouvrir la caméra {self.NUMCAM}")
            sys.exit(1)
        rospy.loginfo(f"✅ 2Caméra {self.NUMCAM} ouverte")

    

    def create_gst_pipeline(self):
        return (
                    f"appsrc is-live=true block=true do-timestamp=true ! videoconvert ! "
                    f"video/x-raw,format=I420,width={FRAME_WIDTH},height={FRAME_HEIGHT},framerate={FRAME_RATE}/1 ! "
                    f"x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! "
                    f"rtph264pay config-interval=1 pt=96 ! "
                    f"udpsink host={self.IP} port={self.PORT} sync=false async=false"
        )


    def create_video_writer(self):
        gst_pipeline = self.create_gst_pipeline()
        self.out = cv2.VideoWriter(gst_pipeline, cv2.CAP_GSTREAMER, 0, FRAME_RATE, (FRAME_WIDTH, FRAME_HEIGHT), True)
        if not self.out.isOpened():
            rospy.loginfo("Erreur : Impossible d'ouvrir le pipeline de streaming 1")
            self.out = None
        else:
            rospy.loginfo("✅ Pipeline GStreamer 1 ouvert")
            pass

    def create_video_writer2(self):
        gst_pipeline = self.create_gst_pipeline()
        self.out2 = cv2.VideoWriter(gst_pipeline, cv2.CAP_GSTREAMER, 0, FRAME_RATE, (FRAME_WIDTH, FRAME_HEIGHT), True)
        if not self.out2.isOpened():
            rospy.loginfo("Erreur : Impossible d'ouvrir le pipeline de streaming 2")
            self.out2 = None
        else:
            rospy.loginfo("✅ Pipeline GStreamer 2 ouvert")
            pass

    def detection_loop(self):
        self.open_camera()
        self.create_video_writer()

        frame_count = 0

        while not self.node.EXIT1:
            rospy.loginfo("A")
            self.cap.grab()

            process_frame = (self.node.STREAM_ON1 and self.out is not None) or self.node.DETECT_ON1
            if not process_frame:
                continue

            ret, frame = self.cap.retrieve()
            if not ret:
                rospy.loginfo("Erreur : Impossible de lire une frame")
                break

            if self.node.STREAM_ON1 and self.out:
                self.out.write(frame)
                rospy.loginfo("B")

            frame_count += 1

        # Libération des ressources
        self.cap.release()
        if self.out:
            self.out.release()
        rospy.loginfo("🛑 Streaming et détection arrêtés pour cam1")


    def detection_loop2(self):
        self.open_camera2()
        self.create_video_writer2()

        frame_count = 0

        while not self.node.EXIT2:
            self.cap2.grab()

            process_frame = (self.node.STREAM_ON2 and self.out2 is not None) or self.node.DETECT_ON2
            if not process_frame:
                continue

            ret, frame = self.cap2.retrieve()
            if not ret:
                rospy.loginfo("Erreur : Impossible de lire une frame")
                break

            if self.node.STREAM_ON2 and self.out2:
                self.out2.write(frame)

            frame_count += 1

        # Libération des ressources
        self.cap2.release()
        if self.out2:
            self.out2.release()
            rospy.loginfo("🛑 Streaming et détection arrêtés pour cam2")


    def start(self):
        """Démarre le thread de détection"""
        self.thread = threading.Thread(target=self.detection_loop)
        rospy.loginfo("Fonction start lancé")
        self.thread.start()
        rospy.loginfo("✅ Start 1")

    def start2(self):
        """Démarre le thread de détection"""
        self.thread2 = threading.Thread(target=self.detection_loop2)
        rospy.loginfo("Fonction start 2 lancé")
        self.thread2.start()
        rospy.loginfo("✅ Start 2")

    def stop(self):
        """Demande l'arrêt et attend la fin du thread"""
        self.node.EXIT1 = True
        if self.thread:
            self.thread.join()
            rospy.loginfo("🛑 Stop 1")

    def stop2(self):
        """Demande l'arrêt et attend la fin du thread"""
        self.node.EXIT2 = True
        if self.thread2:
            self.thread2.join()
            rospy.loginfo("🛑 Stop 2")


