import struct
import subprocess

CMD_NOCMD = 0
CMD_STARTSTREAM = 1
CMD_STOPSTREAM = 2

class Commande:
    def __init__(self):
        self.type = 0
        self.data = b''

    def getTypeCommande(self):
        if len(self.data) > 1:
            return int.from_bytes(self.data[1:2], byteorder='big')
        else:
            return 0

    def SetStartStream(self, ip, port, numcam):
        addr = ip.split('.')
        self.data = bytes([0x04])  # préfixe de trame
        self.data += bytes([CMD_STARTSTREAM])
        self.data += bytes([int(addr[0]), int(addr[1]), int(addr[2]), int(addr[3])])
        self.data += struct.pack('HB', port, numcam)

    def SetStopStream(self, ip, port, numcam):
        addr = ip.split('.')
        self.data = bytes([0x04])  # préfixe de trame
        self.data += bytes([CMD_STOPSTREAM])
        self.data += bytes([int(addr[0]), int(addr[1]), int(addr[2]), int(addr[3])])
        self.data += struct.pack('HB', port, numcam)

    def GetParamStream(self):
        (a, b, c, d, port, numcam) = struct.unpack('BBBBHB', self.data[2:])
        ip = f"{a}.{b}.{c}.{d}"
        return ip, port, numcam

    def StartStream(self):
        REMOTE_IP, REMOTE_PORT, numcam = self.GetParamStream()
        DEVICE = "/dev/video" + str(numcam)

        cmd_open = (
            "gst-launch-1.0 -v v4l2src device=" + DEVICE +
            " ! video/x-raw ! videoconvert ! queue max-size-buffers=1 " +
            "! x264enc tune=zerolatency speed-preset=superfast " +
            "! rtph264pay ! udpsink host=" + REMOTE_IP +
            " port=" + str(REMOTE_PORT) + " sync=false"
        )

        process = subprocess.Popen(cmd_open.split())
        return process



if __name__ == "__main__":
    d = Commande()
    d.SetStartStream('192.168.0.133', 5000, 0)

    print("[TEST] Commande brute envoyée :", d.data)
    print("[TEST] Type de commande :", d.getTypeCommande())
    print("[TEST] Paramètres extraits :", d.GetParamStream())

    d.StartStream()
