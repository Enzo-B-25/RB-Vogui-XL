#!/usr/bin/env python3

import socket
import time
import math
import rospy
from sensor_msgs.msg import Joy

# Adresse IP et port du robot
robot_ip = "192.168.0.210"
robot_port = 30002

# Initialisation de ROS
rospy.init_node('joy_to_robot', anonymous=True)

# Paramètres
delai = 0.35  # délai entre chaque commande (s)
vitesse = 0.6  # vitesse de mouvement
scaling_factor = 0.05  # Déplacement (10 cm max)
rotation_factor = 0.1  # Rotation (0.1 rad ≈ 5.7°)

gripper_open = True

# Positions prédéfinies
list1_degrees = [95.20, -64.09, -158.63, -13.60, 89.72, -105.87]
base = [math.radians(angle) for angle in list1_degrees]

list2_degrees = [265.18, -59.50, -97.04, -148.83, -96.63, 67.88]
arriere = [math.radians(angle) for angle in list2_degrees]

list3_degrees = [4.25, -80.48, 116.25, -124.80, 55.47, 154.40]
app_g = [math.radians(angle) for angle in list3_degrees]

list4_degrees = [-2.5, -86.87, -123.56, -57.82, -51.75, -20.65]
app_d = [math.radians(angle) for angle in list4_degrees]


previous_buttons = None
previous_axes = None

last_command_time = time.time()
command_interval = 0.1  # Délai de 100ms entre chaque envoi

# Connexion au robot
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((robot_ip, robot_port))
rospy.loginfo("✅ Connexion réussie au robot.")



def joy_callback(data):
    global gripper_open, previous_buttons, previous_axes

    if previous_buttons is None:
        previous_buttons = data.buttons  # Initialisation
    if previous_axes is None:
        previous_axes = data.axes  # Initialisation

    # Détection du front montant (appui du bouton)
    def is_button_pressed(index):
        return data.buttons[index] == 1 and previous_buttons[index] == 0

    # Détection du changement d'axe (évite le spam si l'axe ne change pas)
    def has_axis_changed(index, threshold=0.05):
        return abs(data.axes[index] - previous_axes[index]) > threshold

    # Récupération des axes
    axis_y = data.axes[0]  # Joystick gauche horizontal → Y
    axis_z = -data.axes[1]  # Joystick gauche vertical → Z
    axis_rx = -data.axes[2]  # Joystick droit horizontal → Rx
    axis_ry = -data.axes[5]  # Joystick droit vertical → Ry

    deadzone = 0.1
    x_movement, rz_rotation = 0, 0
    y_movement = axis_y * scaling_factor if abs(axis_y) > deadzone and has_axis_changed(0) else 0
    z_movement = axis_z * scaling_factor if abs(axis_z) > deadzone and has_axis_changed(1) else 0
    rx_rotation = axis_rx * rotation_factor if abs(axis_rx) > deadzone and has_axis_changed(2) else 0
    ry_rotation = axis_ry * rotation_factor if abs(axis_ry) > deadzone and has_axis_changed(5) else 0

    # Croix directionnelle pour X et Rz (vérifie le changement)
    if data.axes[10] == 1 and has_axis_changed(10):  # Croix haut → X+
        x_movement = scaling_factor
    elif data.axes[10] == -1 and has_axis_changed(10):  # Croix bas → X-
        x_movement = -scaling_factor
    if data.axes[9] == -1 and has_axis_changed(9):  # Croix droite → Rz-
        rz_rotation = -rotation_factor
    elif data.axes[9] == 1 and has_axis_changed(9):  # Croix gauche → Rz+
        rz_rotation = rotation_factor

    # Mouvements automatiques (boutons)
    if is_button_pressed(0):  # Bouton carré → position de base
        send_move_command(base)
        rospy.loginfo("Retour à la position de base")
    elif is_button_pressed(1):  # Bouton croix → position arrière
        send_move_command(arriere)
        rospy.loginfo("Mouvement vers la position arrière")
    elif is_button_pressed(5):  # Bouton R1 → appui gauche
        send_move_command(app_g)
        rospy.loginfo("Mouvement vers appui gauche")
    elif is_button_pressed(4):  # Bouton L1 → appui droit
        send_move_command(app_d)
        rospy.loginfo("Mouvement vers appui droit")

    # Arrêt du programme
    if is_button_pressed(2):  # Bouton cercle
        print("🔴 Arrêt du programme.")
        sock.close()
        rospy.signal_shutdown("Arrêt manuel")
        
        
    # Envoi du mouvement uniquement si au moins un axe a changé
    if any([x_movement, y_movement, z_movement, rx_rotation, ry_rotation, rz_rotation]):
        command = f"""
        def move_robot():
            movel(pose_trans(get_actual_tcp_pose(), p[{x_movement}, {y_movement}, {z_movement}, {rx_rotation}, {ry_rotation}, {rz_rotation}]), v=0.3, a=1)
        end
        move_robot()
        """
        rospy.loginfo(f"🔄 Déplacement demandé : X={x_movement}m, Y={y_movement}m, Z={z_movement}m, Rx={rx_rotation}rad, Ry={ry_rotation}rad, Rz={rz_rotation}rad")
        sock.sendall(command.encode('utf-8'))
        time.sleep(delai)

        # Mise à jour de l'état précédent UNIQUEMENT SI une commande a été envoyée
        previous_axes = data.axes

    # Mise à jour des boutons
    previous_buttons = data.buttons

def send_move_command(position):
    command = f"""
    def move_robot():
        movej({position}, v={vitesse})
    end
    move_robot()
    """
    sock.sendall(command.encode('utf-8'))
    time.sleep(delai)

def open_gripper():
    global gripper_open
    command = """
    def pince():
        set_digital_out(1, False)
        sleep(0.5)
        set_digital_out(0, True)
        sleep(1)
        set_digital_out(0, False)
    end
    pince()
    """
    rospy.loginfo("🤖 🖐️ Ouverture de la pince")
    gripper_open = True
    sock.sendall(command.encode('utf-8'))

# Abonnement au topic /joy
rospy.Subscriber("/robot/joyArm", Joy, joy_callback)
rospy.spin()

# Fermeture propre
open_gripper()
sock.close()
rospy.loginfo("🔴 Connexion fermée.")
