#!/usr/bin/env python3

import rospy
import socket
import subprocess
import struct
from std_msgs.msg import String

# Adresse IP et port local pour écouter les commandes
LOCAL_IP = "0.0.0.0"  # Écoute sur toutes les interfaces
LOCAL_PORT = 5001
DEVICE = "/dev/video2"  # Modifier si nécessaire

# Création du socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LOCAL_IP, LOCAL_PORT))

# Initialisation du nœud ROS
rospy.init_node('udp_stream_server', anonymous=True)
status_pub = rospy.Publisher('stream_status', String, queue_size=10)
rospy.loginfo("Noeud UDP Stream Server démarré")

# Variable pour stocker le processus GStreamer
process = None

while not rospy.is_shutdown():
    try:
        # Réception du message du client
        rospy.loginfo("Coucou")
        #data, addr = sock.recvfrom(1024)
        #rospy.loginfo(f"Message reçu de {addr}: {data}")

        # Décodage du message
        #entier_received, remote_ip_bytes, remote_port = struct.unpack("!i 13s i", data)
        #remote_ip = remote_ip_bytes.decode().strip('\x00')
        #rospy.loginfo(f"IP client: 192.168.0.172, Port client: 5001")

        if process is None:  # Vérifie si le processus est déjà en cours
            cmd_open = [
                "gst-launch-1.0", "-v", "v4l2src", f"device=/dev/video2",
                "!", "video/x-raw,framerate=20/1",
                "!", "videoscale",
                "!", "clockoverlay", "time-format=%H:%M:%S",
                "!", "videoconvert",
                "!", "x264enc", "tune=zerolatency", "bitrate=500", "speed-preset=superfast",
                "!", "rtph264pay",
                "!", "udpsink host=192.168.0.172 port=5001"
            ]
            rospy.loginfo("Démarrage du stream...")
            process = subprocess.Popen(cmd_open, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            response = "Le stream est lancé"

        else:
            response = "Le stream est déjà actif"
        entier_received = 1

        if entier_received == 2:  # Arrêter le streaming
            if process:
                rospy.loginfo("Arrêt du stream...")
                process.terminate()
                process.wait()
                process = None
                response = "Le stream est arrêté"
            else:
                response = "Aucun stream en cours"

        else:
            response = "Commande inconnue"

        # Envoi de la réponse au client
        sock.sendto(response.encode(),192.168.0.172)
        status_pub.publish(response)
        rospy.loginfo(response)

    except Exception as e:
        rospy.logerr(f"Erreur: {e}")

# Fermeture du socket
sock.close()
