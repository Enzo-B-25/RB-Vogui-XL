#!/usr/bin/env python3

import socket
import time
import math
import rospy
from sensor_msgs.msg import Joy
from ur_msgs.msg import IOStates

# Adresse IP et port du robot
robot_ip = "192.168.0.210"
robot_port = 30002

# Initialisation de ROS
rospy.init_node('joy_to_robot', anonymous=True)

# ---------------- Paramètres --------------------------------

delai = 0.25  # délai entre chaque commande (s)
vitesse = 0.30  # vitesse de mouvement
scaling_factor = 0.02  # Déplacement (2 cm max)
rotation_factor = 0.10  # Rotation (0.10 rad)
lecture_topic = 0.25  # secondes pour réactiver l'écoute du joystick
joy_subscriber = None  # Variable globale pour stocker l'abonnement

# --------------- Positions prédéfinies ----------------------

#base
list1_degrees = [95.20, -64.09, -158.63, -13.60, 89.72, -105.87]
base = [math.radians(angle) for angle in list1_degrees]

#periscope
list2_degrees = [92.03,-84.67,-87.37,-55.68,89.24,-108.78]
periscope = [math.radians(angle) for angle in list2_degrees]

#appui gauche
list3_degrees = [183.98,-138.74,-113.37,-17.30,53.16,-20.93]
app_g = [math.radians(angle) for angle in list3_degrees]

#appui droit
list4_degrees = [178.20,-44.0,112.81,-163.08,-54.51,158.20]
app_d = [math.radians(angle) for angle in list4_degrees]

#---------------------------------------------------------------

# Connexion au robot
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((robot_ip, robot_port))
rospy.loginfo("✅ Connexion réussie au robot.")



def gripper_callback(data):
    global statePin, gripper_subscriber
    statePin = data.digital_out_states #tableau des etats des pins
    
    if gripper_subscriber is not None:
        gripper_subscriber.unregister()
        gripper_subscriber = None


def joy_callback(data):
    global joy_subscriber
    # Vérifier si un bouton est pressé
    if not any(data.buttons) and not any(data.axes):  # Aucun bouton pressé → Ignorer
        return
    
    # Désabonner temporairement pour éviter les appels répétés
    if joy_subscriber is not None:
        joy_subscriber.unregister()
        joy_subscriber = None
        
# --------------  Axes de mouvement ----------------------------

    axis_y = -data.axes[0]  # Joystick gauche horizontal → Y
    axis_z = data.axes[1]  # Joystick gauche vertical → Z
    axis_rx = data.axes[2] # Joystick droit horizontal → Rx
    axis_ry = data.axes[5] # Joystick droit vertical → Ry
    
    x_movement, rz_rotation = 0, 0
    
    deadzone = 0.1
    y_movement = axis_y * scaling_factor if abs(axis_y) > deadzone else 0
    z_movement = axis_z * scaling_factor if abs(axis_z) > deadzone else 0
    rx_rotation = axis_rx * rotation_factor if abs(axis_rx) > deadzone else 0
    ry_rotation = axis_ry * rotation_factor if abs(axis_ry) > deadzone else 0
    
# ------------  Croix directionnelle pour X et Rz --------------


    if data.axes[10]==1:  # Croix haut → X+
        x_movement = scaling_factor
    elif data.axes[10]==-1:  # Croix bas → X-
        x_movement = -scaling_factor
    if data.axes[9]==1:  # Croix droite → Rz-
        rz_rotation = -rotation_factor
    elif data.axes[9]==-1:  # Croix gauche → Rz+
        rz_rotation = rotation_factor
    rospy.loginfo(data.axes)
    
    
# ------------  Mouvements automatiques  ----------------------

    if data.buttons[0]:  # Bouton carré → position de base
        send_move_command(base)
        rospy.loginfo("Retour à la position de base")
    elif data.buttons[1]:  # Bouton croix → position arrière
        send_move_command(periscope)
        rospy.loginfo("Mouvement en position suricate")
    elif data.buttons[10]:  # appui joystick gauche → appui gauche
        send_move_command(app_g)
        rospy.loginfo("Mouvement vers appui gauche")
    elif data.buttons[11]:  # appui joystick droit → appui droit
        send_move_command(app_d)
        rospy.loginfo("Mouvement vers appui droit")
        
#----------------- Controle de la pince -------------------------

    if data.buttons[3]:
    	if statePin[0].state == False : #verifie etat du pin 0
    		command = f"""
            	def move_pince():
            		set_digital_out(1, False)        	
            	end
            	move_pince()
            	"""
    		sock.sendall(command.encode('utf-8'))
    		time.sleep(0.5)    	

    		command = f"""
            	def move_pince():
            		set_digital_out(0, True)        	
            	end
            	move_pince()
            	"""
    		sock.sendall(command.encode('utf-8'))
    		rospy.loginfo("Ouverture de la pince")
    		
    	elif statePin[1].state == False: #verifie etat du pin 1
        
    		command = f"""
            	def move_pince():
            		set_digital_out(0, False)        	
            	end
            	move_pince()
            	"""
    		sock.sendall(command.encode('utf-8'))
    		time.sleep(0.5)
    		command = f"""
            	def move_pince():
            		set_digital_out(1, True)        	
            	end
            	move_pince()
            	"""
    		sock.sendall(command.encode('utf-8'))
    		rospy.loginfo("Fermeture de la pince")
    		    
    	else :
         	rospy.loginfo("Erreur lors du controle de la pince, etat des sorties inconnu")   
    
    	# Réactiver l'écoute du joystick après un délai
    	rospy.Timer(rospy.Duration(lecture_topic), reactivate_gripper_listener, oneshot=True)
      
#---------------------------------------------------------------

    # Vérification de l'arrêt
    #if data.buttons[2]:  # Bouton cercle
    #    rospy.loginfo("🔴 Arrêt du programme.")
     #   sock.close()
     #   rospy.signal_shutdown("Arrêt manuel")
        
    # Envoi du mouvement uniquement si au moins un axe a changé
    if any([x_movement, y_movement, z_movement, rx_rotation, ry_rotation, rz_rotation]):
        command = f"""
        def move_robot():
            movej(pose_trans(get_actual_tcp_pose(), p[{x_movement}, {y_movement}, {z_movement}, {rx_rotation}, {ry_rotation}, {rz_rotation}]), v= {vitesse}, a=1)
        end
        move_robot()
        """
        rospy.loginfo(f"🔄 Déplacement demandé : X={x_movement}m, Y={y_movement}m, Z={z_movement}m, Rx={rx_rotation}rad, Ry={ry_rotation}rad, Rz={rz_rotation}rad")
        sock.sendall(command.encode('utf-8'))
        time.sleep(delai)



    # Réactiver l'écoute du joystick après un délai
    rospy.Timer(rospy.Duration(lecture_topic), reactivate_joy_listener, oneshot=True)
    


def send_move_command(position):
    command = f"""
    def move_robot():
        movej({position}, v={vitesse})
    end
    move_robot()
    """
    sock.sendall(command.encode('utf-8'))
    time.sleep(delai)
    

#---------------------------------------------------------


def reactivate_joy_listener(event):
    """Réactive l'écoute du joystick après un certain délai."""
    global joy_subscriber
    rospy.loginfo("🕹️ Réactivation de l'écoute du joystick...")
    joy_subscriber = rospy.Subscriber("/robot/joyArm", Joy, joy_callback)
    
    
def reactivate_gripper_listener(event):
    """Réactive l'écoute du joystick après un certain délai."""
    global gripper_subscriber
    rospy.loginfo(" Réactivation de l'écoute des pins de la pince...")
    joy_subscriber = rospy.Subscriber("/robot/arm/ur_hardware_interface/io_states", IOStates, gripper_callback)

# Premier abonnement
joy_subscriber = rospy.Subscriber("/robot/joyArm", Joy, joy_callback)

#Abonnement aux etats des sorties digitales du bras 
gripper_subscriber = rospy.Subscriber("/robot/arm/ur_hardware_interface/io_states", IOStates, gripper_callback)


rospy.spin()

# Fermeture propre

sock.close()
rospy.loginfo("🔴 Connexion fermée.")

