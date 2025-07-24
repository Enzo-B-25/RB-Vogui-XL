import struct
import subprocess

CMD_NOCMD=0
CMD_STARTSTREAM=1
CMD_STOPSTREAM=2

class Commande:
    def __init__(self):
        self.type=0
        self.data=''

    
    def getTypeCommande(self):
        if len(self.data)>0:
            # print(type(self.data[0]))
            # return int(self.data[0])
            return int.from_bytes(self.data[0], byteorder='big')
        else:
            return 0
    
    def SetStartStream(self, ip,port,numcam):
        addr=ip.split('.')
        self.data=bytes([CMD_STARTSTREAM])
        self.data+=bytes([int(addr[0])])+bytes([int(addr[1])])+bytes([int(addr[2])])+bytes([int(addr[3])])
        self.data+=struct.pack('HB',port,numcam)
    
    def GetParamStream(self):
        # (a,b,c,d,port,numcam)=struct.unpack('BBBBHB',self.data[1:])
        (a,b,c,d,port,numcam)=struct.unpack('BBBBHB',b''.join((self.data[1:])))
        ip=str(a)+'.'+str(b)+'.'+str(c)+'.'+str(d)
        return ip,port,numcam

    def SetStopStream(self, ip,port,numcam):
        addr=ip.split('.')
        self.data=bytes([CMD_STOPSTREAM])
        self.data+=bytes([int(addr[0])])+bytes([int(addr[1])])+bytes([int(addr[2])])+bytes([int(addr[3])])
        self.data+=struct.pack('HB',port,numcam)
    

    def StartStream(self):
        (REMOTE_IP,REMOTE_PORT,numcam)=self.GetParamStream()
        DEVICE="/dev/video"+str(numcam)
        # cmd_open = "gst-launch-1.0 -v v4l2src device=" + DEVICE + " ! video/x-raw,framerate=20/1 ! videoscale ! clockoverlay time-format=\"%H:%M:%S\" ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=" + REMOTE_IP + " port="+str(REMOTE_PORT)
    
        # cmd_open= "gst-launch-1.0 -v v4l2src device=" +DEVICE + " ! video/x-raw ! clockoverlay time-format=\"%H:%M:%S\"  ! videoconvert ! queue max-size-buffers=1 ! x264enc tune=zerolatency speed-preset=superfast ! rtph264pay ! udpsink host="+REMOTE_IP+" port="+str(REMOTE_PORT)+" sync=false"
        cmd_open= "gst-launch-1.0 -v v4l2src device=" +DEVICE + " ! video/x-raw ! videoconvert ! queue max-size-buffers=1 ! x264enc tune=zerolatency speed-preset=superfast ! rtph264pay ! udpsink host="+REMOTE_IP+" port="+str(REMOTE_PORT)+" sync=false"
        process= subprocess.Popen(cmd_open.split())
        return process

    


d=Commande()
d.SetStartStream('192.168.0.133',1024,0)
# print(d.data)
# print(d.GetStartStream())
d.StartStream()

