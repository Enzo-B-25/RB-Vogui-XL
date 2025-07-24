import socket
import subprocess
import os
import struct

#Global variables
STREAM_ON=1
STREAM_OFF=0
stream_state = STREAM_OFF

# adresse IP et port local
LOCAL_IP = "192.168.0.172"
LOCAL_PORT = 5001

DEVICE = "/dev/video2"

# création du socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LOCAL_IP, LOCAL_PORT))

while True:
    # réception du message du client
    data, addr = sock.recvfrom(1024)

    # conversion du message en entier
    entier_received, REMOTE_IP, REMOTE_PORT = struct.unpack("!i 13s i", data)
    REMOTE_IP=REMOTE_IP.decode()

    #Switch case
    if entier_received == 1:    #Lancement du stream FAIRE ATTENTION DEV VIDEO ET ADRESS IP PC
        if stream_state==STREAM_OFF:
            cmd_open = "gst-launch-1.0 -v v4l2src device=" + DEVICE + " ! video/x-raw,framerate=20/1 ! videoscale ! clockoverlay time-format=\"%H:%M:%S\" ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=" + REMOTE_IP + " port="+str(REMOTE_PORT)
            process = subprocess.Popen(cmd_open.split())
            stream_state = STREAM_ON
            # résultat 
            reponse = "Le stream est lancé";
        else:
            reponse = "Le stream est DEJA lancé"
    elif entier_received == 2:  #Arret du stream
        process.terminate()
        stream_state = STREAM_OFF
        # résultat 
        reponse = "Le stream est arreté";
    else:
        print("Entier invalide")

    # envoi de la réponse au client
    sock.sendto(reponse.encode(), addr)

# fermeture du socket
sock.close()
