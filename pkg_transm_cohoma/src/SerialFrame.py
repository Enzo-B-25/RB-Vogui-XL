import serial
import struct
import rospy
from sensor_msgs.msg import Joy,NavSatFix,BatteryState,Imu
from std_msgs.msg import String, ByteMultiArray


#Types struct
# > little-endian (1023 = 0x03ff   : ff 03 )
# < big endian    (1023 = 0x03ff   : 03 ff )  
# b : int8
# B : uint8
# d : float64
# f : float32
# H : uint16
# q : int64


TOSEND_NO=0
TOSEND_TELEM=2
TOSEND_JOY=3
TOSEND_CMD=4
TOSEND_POSE=5

TYPE_KEEPALIVE=1
TYPE_TELEMETRY=2
TYPE_JOY=3
TYPE_CMD=4
TYPE_CIBLE=6
TYPE_QRCODE=7
#TYPE_LANCE_STREAM_V=4
#TYPE_LANCE_STREAM_H=5
#TYPE_ARRET_STREAM_V=6
#TYPE_ARRET_STREAM_H=7
#TYPE_QRCODE=8
TYPE_DECOLLAGE=9
TYPE_ATTERISSAGE=10
TYPE_HOLD=11
TYPE_ALTITUDE=12
TYPE_GOTO_GPS=13
TYPE_ARM=14
TYPE_DISARM=15
TYPE_MANUEL=16
TYPE_CONNECT=17
TYPE_DETECT_H=18
TYPE_DETECT_V=19
TYPE_GEOFENCE=20
TYPE_RETURN=21


B_CROIX=1
B_ROND=2
B_TRIANGLE=3
B_CARRE=0
B_L1=4
B_R1=5
B_L2=6
B_R2=7
B_SHARE=8
B_OPT=9 
B_PS=12
B_JL=10
B_JR = 11
B_CLIC = 13


J_LH=0
J_LV=1
J_L2=3
J_RH=2
J_RV=5
J_R2=4

J_CROIX_H = 9
J_CROIX_V = 10



class SerialFrame:
    TOSEND_NO=0

    def __init__(self, port, baudrate=9600, start_byte=0xFF, checksum=True):
        """
        Initialise le port série et les paramètres de trame.
        
        Args:
        - port (str): Le port série (ex : 'COM3' ou '/dev/ttyUSB0').
        - baudrate (int): La vitesse de transmission (default: 9600).
        - start_byte (int): Le byte de début de trame (default: 0x02).
        - checksum (bool): Active ou désactive la vérification de la somme de contrôle.
        """
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.serial_port.reset_input_buffer()
        self.start_byte = start_byte
        self.checksum = checksum
        self.buffer = bytearray()

    def read_frame(self):
        """
        Lit les données depuis le port série et identifie les trames complètes basées sur la taille.
        
        Returns:
        - dict ou None: Dictionnaire contenant les données de la trame si complète et valide.
        """
        #rospy.loginfo("Test_1")
        while self.serial_port.in_waiting:
            #rospy.loginfo("Test_2")
            try:
                byte = self.serial_port.read(1)
                if byte == b'':
                    rospy.logwarn("Serial port returned empty byte — possible disconnection.")
                    self.handle_disconnect()
                    return None
            except serial.SerialException as e:
                rospy.logerr(f"Serial exception: {e}")
                self.handle_disconnect()
                return None

            self.buffer.extend(byte)
            
            # Vérifie si une trame complète est disponible
            if len(self.buffer) >= 2:  # start_byte + taille (1 octet)
                #rospy.loginfo("Test_3")
                # Vérifier le byte de début
                if self.buffer[0] != self.start_byte:
                    self.buffer.pop(0)
                    #rospy.loginfo("Test_4")
                    continue

                # Lire la longueur de la trame
                frame_length = self.buffer[1]
                
                # Vérifier si le buffer contient toute la trame
                if len(self.buffer) >= 2 + frame_length:
                    #rospy.loginfo("Test_5")
                    # Extraire la trame complète sans les bytes de début et de longueur
                    frame = self.buffer[2:2 + frame_length]
                    # print(self.buffer)
                    # print(frame)
                    self.buffer = self.buffer[2 + frame_length:]  # Effacer la trame du buffer
                    
                    # Vérifier la somme de contrôle si activée
                    if self.checksum and not self.verify_checksum(frame):
                        print("Erreur de somme de contrôle")
                        continue
                    
                
                    return frame[:-1] # pour enlever le cheksum final
                    
        
        return None

    def send_frame(self, frame):
        """
        Envoie une trame via le port série, incluant la taille et le checksum.
        
        Args:
        - data (dict): Dictionnaire de données à envoyer.
        """
        
        # Calculer la longueur de la trame (1 byte)
        frame_length = struct.pack("B", len(frame) + 1)  # +1 pour inclure le checksum
        
        # Calculer le checksum de la trame sans le start_byte ni frame_length
        checksum = sum(frame) & 0xFF
        frame_with_checksum = frame + bytes([checksum])
        
        # Construire la trame complète
        complete_frame = bytes([self.start_byte]) + frame_length + frame_with_checksum
        
        # Envoi de la trame sur le port série
        self.serial_port.write(complete_frame)


    def verify_checksum(self, frame):
        """
        Vérifie la somme de contrôle d'une trame.
        
        Args:
        - frame (bytes): La trame à vérifier.
        
        Returns:
        - bool: True si la somme de contrôle est correcte, sinon False.
        """
        if len(frame) < 2:
            return False
        data, received_checksum = frame[:-1], frame[-1]
        calculated_checksum = sum(data) & 0xFF  # Somme des octets sur 1 byte
        return calculated_checksum == received_checksum



    def encode_frame(self, data):
        """
        Encode un dictionnaire de données en une trame binaire avec somme de contrôle.
        
        Args:
        - data (dict): Dictionnaire avec les données à encoder.
        
        Returns:
        - bytes: La trame binaire prête à être envoyée.
        """
        sensor_id = data.get("sensor_id", 0)
        measurement = data.get("measurement", 0.0)
        frame = struct.pack('>Hf', sensor_id, measurement)
        
        # Calcul de la somme de contrôle si activée
        if self.checksum:
            checksum = sum(frame) & 0xFF
            frame += bytes([checksum])
        
        return frame

    ######################        Telemetry  (GPS+batt ) #######################################
    # TODO IMU

    
     
    def encode_telemetry(self,GPS,Batt,Imu):
        frame=struct.pack(">bqbdddbfffff",
        TYPE_TELEMETRY,
        GPS.header.stamp.secs,
        GPS.status.status,
        GPS.latitude,
        GPS.longitude,
        GPS.altitude,
        int(Batt.level),
        Batt.voltage,
        Imu.orientation.x,
        Imu.orientation.y,
        Imu.orientation.z,
        Imu.orientation.w)

        return frame

    def decode_telemetry(self,frame):
        msg_GPS=NavSatFix()
        msg_Batt=BatteryState()
        msg_IMU = Imu()
        (tt,msg_GPS.header.stamp.sec,msg_GPS.status.status,
        msg_GPS.latitude,msg_GPS.longitude,msg_GPS.altitude,
        percentage,msg_Batt.voltage)=struct.unpack(">bqbdddbf",frame)
        msg_Batt.percentage=percentage/100
        return msg_GPS,msg_Batt,msg_IMU
    
    ######################        JOY  ############################################

    def encode_joy(self,Joy):
        button=Joy.buttons[B_CROIX]+(Joy.buttons[B_ROND]<<1)+(Joy.buttons[B_TRIANGLE]<<2)
        button+=(Joy.buttons[B_CARRE]<<3)+(Joy.buttons[B_L1]<<4)+(Joy.buttons[B_R1]<<5)
        button+=(Joy.buttons[B_CROIX_HAUT]<<6)+(Joy.buttons[B_CROIX_BAS]<<7)+(Joy.buttons[B_SAHRE]<<8)
        button+=(Joy.buttons[B_OPT]<<9)+(Joy.buttons[B_PS]<<10)+(Joy.buttons[B_CROIX_GAUCHE]<<11)+(Joy.buttons[B_CROIX_DROIT]<<12)
        button+=(Joy.buttons[B_JOY_DROIT]<<13)+(Joy.buttons[B_JOY_GAUCHE]<<14)

        frame=struct.pack('>bffffffH',TYPE_JOY,
                                Joy.axes[J_LH],
                                Joy.axes[J_LV],
                                Joy.axes[J_RH],
                                Joy.axes[J_RV],
                                Joy.axes[J_L2],
                                Joy.axes[J_R2],
                                button)
        return frame

    def decode_joy(self,frame):
        print(f"Taille du frame reçu : {len(frame)} octets")

        msg=Joy()
        msg.axes=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        msg.buttons=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        (tt,msg.axes[J_LH],msg.axes[J_LV],msg.axes[J_RH],
        msg.axes[J_RV],msg.axes[J_L2],msg.axes[J_R2],button)=struct.unpack('>bffffffH',frame)
        msg.buttons[B_CROIX]=button & 1
        msg.buttons[B_ROND]=(button >> 1) &1
        msg.buttons[B_CARRE]=(button >> 2) &1
        msg.buttons[B_TRIANGLE]=(button >> 3) &1
        msg.buttons[B_SHARE]=(button >> 4) &1
        msg.buttons[B_PS]=(button >> 5) &1
        msg.buttons[B_OPT]=(button >> 6) &1
        msg.buttons[B_JL]=(button >> 7) &1
        msg.buttons[B_JR]=(button >> 8) &1
        msg.buttons[B_L1]=(button >> 9) &1
        msg.buttons[B_R1]=(button >> 10) &1
        msg.buttons[B_CLIC]=(button >> 15) &1
    
        # Mapping des axes pour labaudr croix directionnelle (D-pad)
        msg.axes[J_CROIX_V] = ((button >> 11) & 1) - ((button >> 12) & 1)  # Haut - Bas
        msg.axes[J_CROIX_H] = ((button >> 14) & 1) - ((button >> 13) & 1)  # Droite - Gauche
    
        return msg

    ######################        CMD  ############################################
    def encode_cmd(self, cmd):
        frame = bytes([TYPE_CMD] + cmd.data)
        return frame

    def decode_cmd(self, frame):
        cmd = ByteMultiArray()
        #cmd.data = list(frame[1:])
        raw_bytes = frame[1:]
        cmd.data = [int(b) if b<128 else b-256 for b in raw_bytes]
        return cmd
        

    def close(self):
        """ Ferme le port série. """
        self.serial_port.close()
