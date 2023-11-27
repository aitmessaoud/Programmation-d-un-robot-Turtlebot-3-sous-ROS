#!/usr/bin/env python3

# Importation des modules nécessaires
import rospy
import sys, termios, tty
import click
from geometry_msgs.msg import Twist 

# Définition des touches du clavier
keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit'}

# Initialisation du noeud
rospy.init_node('mybot_teleop')

# Définition du topic de publication et de la taille de la file
pub  = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

# Définition de la fréquence de publication
rate = rospy.Rate(10)

if __name__ == '__main__':
    
    # Initialisation de la variable de publication
    publish =False

    try:    
        # Récupération des paramètres de lancement (linear_scale et angular_scale)
        # Si aucun paramètre n'est spécifié, on prend comme valeur par défaut 1.0
        linear_scale  =  rospy.get_param('~linear_scale', 1.0)
        angular_scale =  rospy.get_param('~angular_scale', 1.0)

        # Création d'un message Twist vide
        msg = Twist()

        while not rospy.is_shutdown():
            # Lecture de la touche pressée par l'utilisateur
            mykey = click.getchar()
            
            # Conversion de la touche pressée en une commande de mouvement
            if mykey in keys.keys():
                char=keys[mykey]
            
            if char == 'up':    # Touche du haut pressée
                # Mouvement vers l'avant
                linear = linear_scale
                angl=0.0
                publish= True            
                
            if char == 'down':  # Touche du bas pressée
                # Mouvement vers l'arrière
                linear = -1*linear_scale
                angl=0.0
                publish= True
                
            if char == 'left':  # Touche de gauche pressée
                # Rotation vers la gauche
                linear = 0.0
                angl=angular_scale
                publish= True
                
            if char == 'right': # Touche de droite pressée
                # Rotation vers la droite
                linear = 0
                angl=-1*angular_scale
                publish= True
                
            if char == "quit":  # Touche 'q' pressée
                # Arrêt du programme
                break
                
            if publish:
                # Publication du message Twist sur le topic "/turtle1/cmd_vel"
                publish=False
                msg.linear.x=linear
                msg.angular.z=angl
                pub.publish(msg)

            # Attente pour respecter la fréquence de publication définie
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

