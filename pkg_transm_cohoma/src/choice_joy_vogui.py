#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time


class ChoiceJoy:
    def __init__(self):
        # Initialisation du noeud ROS
        rospy.init_node('choice_joy', anonymous=True)

        # Souscription au topic des messages de la manette
        self.choix_joy = rospy.Subscriber('/robot/joyRecu', Joy, self.read_joy)

        # Publication pour le moteur Vogui ou le bras
        self.pub_joy_vogui_motor = rospy.Publisher("/robot/joyMoteur", Joy, queue_size=1)
        self.pub_joy_arm_vogui = rospy.Publisher("/robot/joyArm", Joy, queue_size=1)

        self.compt = 0  # Indique quel mode est actif : 0 = moteur, 1 = bras
        self.prev_button_state = 0  # État précédent du bouton B_OPT
        self.last_check_time = time.time()  # Dernier moment où le bouton a été vérifié

        # Index du bouton B_OPT (à corriger selon votre manette)
        self.B_OPT = 9  # Bouton Share

        rospy.loginfo("Noeud de sélection pour le contrôle de Vogui ou bras initialisé.")

        # Boucle périodique pour vérifier le bouton et publier les commandes
        self.rate = rospy.Rate(10)  # 10 Hz
        self.run()

    def read_joy(self, msg):
        """Callback pour lire l'état actuel des boutons de la manette."""
        self.joy_msg = msg  # Stocke le dernier message Joy reçu


    def check_button(self):
        """Vérifie l'état du bouton B_OPT toutes les 0.2 secondes."""
        current_time = time.time()

        if hasattr(self, 'joy_msg'):  # Assure que `self.joy_msg` a été reçu
            button_state = self.joy_msg.buttons[self.B_OPT]

            # Flanc montant : le bouton passe de relâché (0) à pressé (1)
            if button_state == 1 and self.prev_button_state == 0:
                self.compt = (self.compt + 1) % 2
                rospy.loginfo(f"Mode de contrôle changé : {'Moteur Vogui' if self.compt == 0 else 'Bras Vogui'}")

            # Mise à jour de l'état précédent
            self.prev_button_state = button_state

    def publish_commands(self):
        """Publie les commandes sur le topic correspondant au mode actif."""
        if hasattr(self, 'joy_msg'):  # Assure que `self.joy_msg` a été reçu
            if self.compt == 0:
                self.pub_joy_vogui_motor.publish(self.joy_msg)
                rospy.loginfo("Commande moteur Vogui publiée.")
            elif self.compt == 1:
                self.pub_joy_arm_vogui.publish(self.joy_msg)
                rospy.loginfo("Commande bras Vogui publiée.")

    def run(self):
        """Boucle principale pour vérifier le bouton et publier les commandes."""
        while not rospy.is_shutdown():
            self.check_button()  # Vérifie l'état du bouton périodiquement
            self.publish_commands()  # Publie les commandes au bon endroit
            self.rate.sleep()


if __name__ == '__main__':
    try:
        ChoiceJoy()
    except rospy.ROSInterruptException:
        pass

