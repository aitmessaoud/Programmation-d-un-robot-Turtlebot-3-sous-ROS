#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  1 23:12:41 2023

@author: Salem
"""


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np 
err = [] # Tableau pour stocker les valeurs de l'erreur  
def stop_robot():
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
def callback(data):
    global cmd_vel_pub
    global err
    # Prendre les mesures de capteur de chaque côté de l'avant du robot
    ranges = data.ranges[0:5] + data.ranges[350:360]
  
    # Trouver la distance la plus proche pour l'avant du robot
    front = min([data.range_max if rng > data.range_max else data.range_min if rng < data.range_min else rng for rng in ranges])
    #front = data.ranges[0]
    # Trouver la distance la plus proche pour le capteur gauche du robot
    left = min([data.range_max if rng > data.range_max else data.range_min if rng < data.range_min else rng for rng in data.ranges[45:100]])    
    #left = data.ranges[45]
    # Trouver la distance la plus proche pour le capteur droit du robot
    right = min([data.range_max if rng > data.range_max else data.range_min if rng < data.range_min else rng for rng in data.ranges[300:330]])    
    #right = data.ranges[360-45]
    # Initialiser la commande de vitesse angulaire et linéaire
    twist = Twist()
    
    # Définir la vitesse de déplacement linéaire et angulaire du robot
    twist.linear.x = rospy.get_param("lineaire_vel",0.08) 
    ang_vel = rospy.get_param("angulaire_vel",0.9) 
    #ang_vel = rospy.get_param("angulaire_vel",2.2) 
    #ang_vel = rospy.get_param("angulaire_vel",0.3)
    print(twist.linear.x )
    # Calculer l'erreur de distance entre le capteur gauche et le capteur droit
    error = left - right
    # Pour chaque vitesse angulaire on stocke les valeurs de l'erreur 
    err.append(abs(error)) 
    # Création du fichier txt contenant les valeurs de l'err pour une vitesse angulaire de 0.9 m/s
    #np.savetxt('test1.challenge2.txt',err)
    
    # Création du fichier txt contenant les valeurs de l'err pour une vitesse angulaire de 0.3 m/s
    #np.savetxt('test2.challenge2.txt',err) 
   #Création du fichier txt contenant les valeurs de l'err pour une vitesse angulaire de 2.2 m/s
    #np.savetxt('test3.challenge2.txt',err) 
    
    
    # Initialiser le seuil de tolérance
    seuil = 0.001 
    
    # Si l'erreur est différente du seuil de tolérance et qu'il n'y a pas d'obstacle à l'avant
    if error != seuil and front > 0.3:
        # Si la distance à gauche est plus grande que la distance à droite
        if left > right: 
           print("turn left")
           # On ajuste la vitesse angulaire en fonction de l'erreur 
           # Ici error > 0 et donc on tourne à gauche 
           twist.angular.z =  ang_vel * float(error)
        # Si la distance à droite est plus grande que la distance à gauche 
        else:
           print("turn right")
           # On ajuste la vitesse angulaire en fonction de l'erreur 
           # Ici error < 0 et donc on tourne à droite 
           twist.angular.z =  ang_vel * float(error)
    
    # Si la distance avant est inférieure à 0.3 mètres, le robot s'arrête et tourne dans la bonne direction
    elif front <= 0.3:
          # Arrêt du mouvement linéaire
          twist.linear.x = 0.0 
          cmd_vel_pub.publish(twist)
          
          # Si plus d'espace à droite qu'à gauche
          if right > left:
            # On tourne à droite 
            twist.angular.z =  -ang_vel #ang_vel*abs(front-0.4)  
            print("sens direct")
          else : 
            # On tourne à gauche
            twist.angular.z =  ang_vel #*ang_vel*abs(front-0.4) 
            print("sens indirect")  
     
    # Publication de la commande de vitesse 
    cmd_vel_pub.publish(twist)
     
    
# Cette fonction permet de s'abonner aux données du capteur laser et de publier les commandes de vitesse pour le robot.
def lds_distance():
    global cmd_vel_pub
    
    # Initialisation du node ROS "corridor" de façon anonyme.
    rospy.init_node('corridor', anonymous=True)
    
    # Souscription aux données du capteur laser pour exécuter la fonction "callback" dès que de nouvelles données sont reçues.
    rospy.Subscriber("scan", LaserScan, callback)
    
    # Initialisation du Publisher "cmd_vel_pub" pour publier les commandes de vitesse sur le topic "/cmd_vel" avec une file d'attente de 10.
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    # Exécution de la fonction "stop_robot" lors de l'arrêt du node ROS.
    rospy.on_shutdown(stop_robot)
    
    # Boucle pour maintenir le node ROS en cours d'exécution.
    rospy.spin()

# Point d'entrée du script.
if __name__ == '__main__':
    try:
        # Appel de la fonction "lds_distance".
        lds_distance()
    except rospy.ROSInterruptException:
        # Gestion d'une éventuelle interruption de l'exécution de ROS.
        pass

