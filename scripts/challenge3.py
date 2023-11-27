#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  3 18:43:22 2023

@author: turtle
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 31 22:49:11 2023

@author: turtle
"""



import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

bridge = CvBridge()
# lecture des valeurs HSV des couleurs passées en paramétre dans le fichier launch 
lh1 = rospy.get_param("/lh1",35)
ls1 = rospy.get_param("/ls1",50)
lv1 = rospy.get_param("/lv1",50)
uh1 = rospy.get_param("/uh1",85)
us1 = rospy.get_param("/us1",255)
uv1 = rospy.get_param("/uv1",255)
bridge = CvBridge()
# Threshold values for green color detection
print(lh1,ls1,lv1)
lower_green = np.array([lh1, ls1, lv1])
upper_green = np.array([uh1, us1, uv1])


cmd_vel_pub= None 
non_contour = False # booléen indiquant si au moins un cylindre vert est détecté 
obstacle_detected = False # booléen indiquant si un obstacle est détecté 
def stop_robot():
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)

def image_callback(msg):
    global non_contour
    global cmd_vel_pub
   
    # Convertir le message ROS Image en image OpenCV
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    
    # Récupérer la hauteur, la largeur et le nombre de canaux de l'image
    h, w, _ = image.shape 

    # Convertir l'image BGR en espace de couleur HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Créer un masque pour extraire les pixels correspondant à la couleur verte
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Trouver les contours dans le masque
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Initialiser la vitesse de déplacement du robot
    vel = 0.8#rospy.get_param('vel',0.8)
    twist = Twist()
     
    
    if len(contours) > 0:
         if len(contours) == 2 :
            # Les deux cylindres verts sont détectés, on se dirige vers le centre des deux cylindres 
            print("nombre de contours : " ,len(contours))
            non_contour = False 
            # Calculer les moments pour chaque contour
            M1 = cv2.moments(contours[0])
            M2 = cv2.moments(contours[1])
             
            if M1['m00'] !=0 and M2['m00']!=0 :
                # Calculer le centroïde pour chaque contour
                 cy1 = int(M1['m01'] / M1['m00'])
                 cx1 = int(M1['m10'] / M1['m00'])
                 cy2 = int(M2['m01'] / M2['m00'])
                 cx2 = int(M2['m10'] / M2['m00'])
                 # Calculer la médiane des deux centroïdes pour obtenir le centre des deux cylindres
                 cx = (cx1+cx2)/2 
                 cy = (cy1+cy2)/2 
                 # Calculer l'erreur en x (distance horizontale entre le centre de l'image et le centre des cylindres)
                 error_x = w/2-cx
                 # Dessiner un cercle autour du centre des cylindres sur l'image pour vérification
                 cv2.circle(image,(int(cx),int(cy)),10,(0,0,255),-1)
                 cv2.imshow('mask',mask) 
                 cv2.imshow('image',image)
                 cv2.waitKey(1)
                 print("deux cylindres détectés")
                 # Publier la commande de mouvement avec une vitesse linéaire proportionnelle à cy et une vitesse angulaire proportionnelle à l'erreur en x
                 twist.linear.x = vel*cy/500
                 twist.angular.z = float(error_x) /50
                  # Publier la commande de mouvement uniquement si aucun obstacle n'est détecté car la priorité est à l'évitement d'obstacles
                 if obstacle_detected == False : cmd_vel_pub.publish(twist)
           
             
                              
         elif len(contours) == 1 : 
         # Un seul cylindre vert est détecté, on se dirige vers ce cylindre
            # Calculer le moment du contour
            M = cv2.moments(contours[0])
            # si le moment est non nul
            if M["m00"] > 0 :
                # on calcule le centroide du contour
                 cx = int(M[ "m10" ] / M[ "m00" ] )
                 cy = int(M[ "m01" ] / M[ "m00" ] )
                 # on calcule l'erreur horizontale
                 error_x = w/2-cx 
                 # on dessine un cercle sur l'image au niveau du centroide
                 cv2.circle(image,(int(cx),int(cy)),10,(0,255,0),-1)
                 # on affiche les images pour le debuggage
                 cv2.imshow('mask',mask) 
                 cv2.imshow('image',image)
                 cv2.waitKey(1)
                 print("un seul cylindre détecté")
                 twist.linear.x = vel*cy/500
                 twist.angular.z = float(error_x) /50
                 print("je publie dans image")
                # Publier la commande de mouvement uniquement si aucun obstacle n'est détecté car la priorité est à l'évitement d'obstacles
                 if obstacle_detected == False : cmd_vel_pub.publish(twist)
        
                           
                 
    else :   
            # si aucun des deux cylindres n'est détecté 
    		# on fait tourner le robot pour chercher la sortie 
            non_contour = True 
            if obstacle_detected == False : 
                 twist.linear.x = 0.1
                 twist.angular.z =  0.3
                 cmd_vel_pub.publish(twist)
                 print("je vois pas la sortie") 
def scan_callback(msg):
    global obstacle_detected
    #  Obtenir les distances du scanneur LIDAR
    ranges = msg.ranges[0:25] + msg.ranges[330:359]
    front = min([rng if rng < 3 else 3 for rng in ranges])
    right = min([rng if rng < 3 else 3 for rng in msg.ranges[233:317]])
    left = min([rng if rng < 3 else 3 for rng in msg.ranges[45:137]])
    # Créer un objet Twist pour le contrôle de la vitesse et de l'orientation     
    twist= Twist()
    # Définir les seuils et les vitesses de rotation pour éviter les obstacles
    seuil = 0.28
    ang_vel = 2.0
    
    # Si un obstacle est détecté devant le robot
    if front < seuil:
       obstacle_detected = True 
       # Afficher un message d'information
       rospy.loginfo("obstacle !!!")
       # Arrêter le robot
       stop_robot()
       # Tourner à droite ou à gauche en fonction de la distance à l'obstacle
       if right < left:
          
          #turn left :
                                                     
           rospy.loginfo("turn left") 
           
           twist.angular.z = ang_vel
           
       else :
           #turn right
           rospy.loginfo("turn right") 
            
           twist.angular.z = -ang_vel
       # Publier la commande de vitesse et d'orientation        
       cmd_vel_pub.publish(twist)     
        
           
    else : 
       # Si aucun obstacle n'est détecté devant le robot
       obstacle_detected= False 
       
        
        
               
            
def main():
    global cmd_vel_pub

    # Initialize the node
    rospy.init_node("line_following")
    rospy.loginfo("main")
    # Subscribe to the image topic
    rospy.Subscriber("/camera/image", Image, image_callback)
    rospy.Subscriber("scan", LaserScan,scan_callback)
    # Initialize the velocity command publisher
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
     # Exécution de la fonction "stop_robot" lors de l'arrêt du node ROS.

    rospy.on_shutdown(stop_robot)
    # Spin the node
    rospy.spin()


if __name__ == '__main__':
    main()
          
            
 
