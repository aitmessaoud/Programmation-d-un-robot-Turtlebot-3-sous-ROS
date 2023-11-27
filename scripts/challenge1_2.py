#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  1 23:12:41 2023

@author: Salem
"""


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Initialisation de l'interface de conversion d'images entre OpenCV et ROS
bridge = CvBridge()
# lecture des valeurs HSV des couleurs passées en paramétre dans le fichier launch 
#Color 1 HSV values (Yellow)
lh1 = rospy.get_param("/lh1",20)
ls1 = rospy.get_param("/ls1",100)
lv1 = rospy.get_param("/lv1",20)

uh1 = rospy.get_param("/uh1",40)
us1 = rospy.get_param("/us1",255)
uv1 = rospy.get_param("/uv1",255)
#Color 2 HSV values (White)
lh2 = rospy.get_param("/lh2",0)
ls2 = rospy.get_param("/ls2",0)
lv2 = rospy.get_param("/lv2",180)

uh2 = rospy.get_param("/uh2",255)
us2 = rospy.get_param("/us2",25)
uv2 = rospy.get_param("/uv2",255)
# Seuils pour la détection des couleurs jaune et blanche
#couleur jaune
lower_yellow = np.array([lh1, ls1, lv1])
upper_yellow = np.array([uh1, us1, uv1])
#couleur blanche
lower_white = np.array([lh2, ls2, lv2])
upper_white = np.array([uh2, us2, uv2])
"""
#couleur rouge
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])
lower_red2 = np.array([170, 100, 100])
upper_red2 = np.array([180, 255, 255])
"""
# Variables pour la détection de la ligne
capteur_left, capteur_right = 4, 4 # Distance en cm entre le robot et les obstacles détectés par les capteurs latéraux
 
obstacle_detected = False # Booléen pour indiquer si le robot a détecté un obstace ou pas (True si oui, False sinon)
turn_right = [] # Tableau de booléens pour indiquer dans quelle direction tourner (à droite)
turn_left =[] # Tableau de booléens pour indiquer dans quelle direction tourner (à gauche)
factor = 40 # Facteur de réduction de la vitesse angulaire
 

def stop_robot():
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    
# Fonction pour suivre la ligne jaune
def follow_yellow_line(contours_yellow, mask_yellow, image, w, vel, dec):
    global err 
    # On trouve le plus grand contour
    biggest_contour_yellow = max(contours_yellow, key=cv2.contourArea)
    M2=  cv2.moments ( biggest_contour_yellow )
    cx2, cy2 = int(M2[ "m10" ] / M2[ "m00" ] ) + dec, int(M2[ "m01" ] / M2[ "m00" ] ) # On calcule le centroïde du contour en décalant le cx  
    error_x = w/2 - cx2 # On calcule l'écart entre le centre de l'image du robot et le centre du contour
    # On affiche le centroïde du contour
    cv2.circle(image,(cx2, cy2), 10, (0,0,255), -1)
    # On affiche les images pour le debug
    cv2.imshow('mask_jaune', mask_yellow)
    cv2.imshow('image',image)
    cv2.waitKey(1)
    # On définit la vitesse linéaire et angulaire du robot en fonction de l'erreur
    vel_linear = vel
    vel_angular = float(error_x) / factor
 
     
    return vel_linear, vel_angular
# Fonction pour suivre la ligne blanche
def follow_white_line(contours_white,mask_white,image,w,vel,dec): 
    global err
    biggest_contour_white = max(contours_white, key=cv2.contourArea)
    M1 = cv2.moments ( biggest_contour_white )
    cx1,cy1 = int(M1[ "m10" ] / M1[ "m00" ] )-dec,int(M1[ "m01" ] / M1[ "m00" ] ) # on calcule le centroide du contour en décalant le cx  
    error_x = w/2-cx1 # on calcule l'ecart entre le centre de l'image du robot et le centre du contour
    # On affiche le centroïde du contour 
    cv2.circle(image,(cx1,cy1),10,(0,0,255),-1)
    cv2.imshow('mask_blanc', mask_white)
    cv2.imshow('image',image)
    cv2.waitKey(1)
    # On définit la vitesse linéaire et angulaire du robot en fonction de l'erreur
    vel_linear = vel
    vel_angular = float(error_x) /factor 
     
    return vel_linear,vel_angular           
def test_contour(contours):
    "Détermine si la ligne est bien détectée"
    try:
        # Recherche du plus grand contour
        biggest_contour = max(contours, key=cv2.contourArea)
        # Calcul des moments d'ordre 0 et 1
        M = cv2.moments(biggest_contour)
        # Si le moment d'ordre 0 est nul, la ligne n'est pas valide
        if M["m00"] == 0:
            valide = False
        else:
            valide = True
        # Retourne la validité de la ligne
        return valide
    except:
        # Si une erreur est survenue, la ligne n'est pas valide
        valide = False
        # Retourne la validité de la ligne
        return valide

def calcule_decalage(contours_white, contours_yellow):
    "Calcule de combien doit-on décaler la ligne pour que le robot soit au centre"
    # Recherche du plus grand contour jaune
    biggest_contour_yellow = max(contours_yellow, key=cv2.contourArea)
    # Calcul des moments d'ordre 0 et 1
    M2 = cv2.moments(biggest_contour_yellow)
    # Calcul des coordonnées du centre du contour jaune
    cx2, cy2 = int(M2["m10"] / M2["m00"]), int(M2["m01"] / M2["m00"])
    
    # Recherche du plus grand contour blanc
    biggest_contour_white = max(contours_white, key=cv2.contourArea)
    # Calcul des moments d'ordre 0 et 1
    M1 = cv2.moments(biggest_contour_white)
    # Calcul des coordonnées du centre du contour blanc
    cx1, cy1 = int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"])
    
    # Calcul du décalage entre les deux centres
    dec = int((cx1 - cx2) / 2)
    print(dec)
   
    
    # Vérification que le décalage est bien dans la plage [70, 90]
    # Le choix de cette plage est fait de sorte que le robot reste toujours entre les deux lignes 
    if dec < 70 or dec > 90:
        dec = 80
    
    # Retourne le décalage
    return dec

def reshape_image(image,factor_h,factor_w) : 
    # Récupérer les dimensions de l'image
    h,w,_=image.shape 
    
    # Définir les zones de recherche d'informations (ici en fonction de h et w)
    search_top = h*factor_h
    search_bot = h
    search_left = w*factor_w
    search_right = w*(1-factor_w)
    
    # Masquer les parties de l'image inutiles pour la détection de la ligne
    image[0:int(search_top), 0:w] = 0
    image[int(search_bot):h, 0:w] = 0
    h ,_,_=image.shape 
    image[0:h, 0:int(search_left)] = 0
    image[0:h, int(search_right):w] = 0
 
                       
def image_callback(msg):
    # Déclaration des variables globales utilisées dans la fonction
    global non_contour
    global cmd_vel_pub
    global turn_right 
    global turn_left
    global capteur_left
    global capteur_right
    global count_lines 
    
    # Convertir le message d'image ROS en image OpenCV
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
     
    # Conserver une copie de l'image originale pour une utilisation ultérieure
    image_originale = np.copy(image)
    # on retaille l'image pour prendre que la zone essentielle 
    reshape_image(image,1/6,1/8) 
    h,w,_ = image.shape
    
    
    # Convertir l'image en espace de couleur HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
     
     
    # Créer des masques pour détecter les couleurs jaune et blanche
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
     
    # Créer un objet Twist pour contrôler les mouvements du robot
    twist = Twist()
      
    # Trouver les contours des zones blanches et jaunes
    contours_white, _ = cv2.findContours(mask_white,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_yellow, _ = cv2.findContours(mask_yellow,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Définir la vitesse d'avancement du robot
    vel = rospy.get_param("vel",0.12) # par défaut on prend 0.12

     
    # Vérifie si des contours blancs ou jaunes sont détectés et s'il n'y a pas d'obstacle détecté
    if (len(contours_white) > 0 or len(contours_yellow) >0) and obstacle_detected == False :

        # Avance tant que l'obstacle n'est pas terminé
        if (capteur_left < 0.17 or capteur_right < 0.17):
            twist.linear.x = vel
            twist.angular.z = 0.0
            print("j'avance")

        # Si le robot peut suivre l'une des deux lignes, calcule le décalage de la ligne pour le centrage du robot
        else:
            valide_white  = test_contour(contours_white)
            valide_yellow = test_contour(contours_yellow)

            # Vérifier si le robot a détecté à la fois la ligne blanche et la ligne jaune
            if valide_white and valide_yellow :
                # Si oui, calculer le décalage entre les deux lignes
                decalage_ligne = calcule_decalage(contours_white,contours_yellow)
            else :
                # Si non, régler le décalage à une valeur par défaut de 75
                decalage_ligne = 75
                
            # Si le robot détecte la ligne blanche, suit la ligne en décalant pour centrer le robot
            if valide_white == True:
                #print("je suis la ligne blanche",decalage_ligne)
                twist.linear.x,twist.angular.z = follow_white_line(contours_white,mask_white,image,w,vel,decalage_ligne)

            # Si le robot détecte la ligne jaune, suit la ligne en décalant pour centrer le robot
            elif valide_yellow == True:
                #print("je suis la ligne jaune")
                twist.linear.x,twist.angular.z = follow_yellow_line(contours_yellow,mask_yellow,image,w,vel,decalage_ligne)

    # Si aucun contour n'est détecté
    else:  
        # Si un obstacle est détecté, tourne à droite ou à gauche pour l'éviter
        if obstacle_detected == True:
            print("obstacle detected") 
            twist.linear.x = 0.00
            #cmd_vel_pub.publish(twist)
            if turn_right[-1] == True:
                print("je tourne à droite")
                twist.angular.z = -0.9
                 
            elif turn_left[-1] == True:
                print("je tourne à gauche")
                twist.angular.z = 0.9
                 

        # Si aucun contour ni obstacle n'est détecté
        else:
            twist.angular.z = 0.9
            print("no contours!!")
    
      
    cmd_vel_pub.publish(twist)
     

    #rate.sleep()    
       
def scan_callback(msg):
    global obstacle_detected # Indique si un obstacle est détecté dans la plage de l'obstacle_range
    global previous_obstacle # Stocke la position de l'obstacle précédent pour éviter de se bloquer dans une boucle d'évitement
    global turn_right # Stocke une valeur booléenne indiquant si le robot doit tourner à droite pour éviter l'obstacle le plus proche
    global turn_left # Stocke une valeur booléenne indiquant si le robot doit tourner à gauche pour éviter l'obstacle le plus proche
    global capteur_left # Stocke la distance de l'obstacle le plus proche détecté par le capteur gauche
    global capteur_right # Stocke la distance de l'obstacle le plus proche détecté par le capteur droit
    
    # Récupérer les distances de la balayage LIDAR
    capteur_front_right = min([msg.range_max if rng > msg.range_max else msg.range_min if rng < msg.range_min else rng for rng in msg.ranges[330:350]]) # Capteur avant droit
    capteur_front_left = min([msg.range_max if rng > msg.range_max else msg.range_min if rng < msg.range_min else rng for rng in msg.ranges[10:30] ]) # Capteur avant gauche
    capteur_left = min([msg.range_max if rng > msg.range_max else msg.range_min if rng < msg.range_min else rng for rng in msg.ranges[60:107]]) # Capteur gauche
    capteur_right = min([msg.range_max if rng > msg.range_max else msg.range_min if rng < msg.range_min else rng for rng in msg.ranges[233:317]]) # Capteur droit
    closest_obstacle = min(capteur_front_right, capteur_front_left)
    
    # Plage considérée pour les obstacles
    obstacle_range = 0.25

    twist = Twist()
    
    # Vérifier si un obstacle se trouve dans la plage de l'obstacle_range
    if closest_obstacle < obstacle_range:
        # Détecté obstacle
        if closest_obstacle == capteur_front_left:
            rospy.loginfo('Obstacle detecté à gauche')
            turn_right.append(True) # Si l'obstacle est détecté à gauche, tourner à droite
            turn_left.append(False)
              
        elif closest_obstacle == capteur_front_right:
            rospy.loginfo('Obstacle detecté à droite')
            turn_left.append(True) # Si l'obstacle est détecté à droite, tourner à gauche
            turn_right.append(False)
          
        cmd_vel_pub.publish(twist)   
          
        obstacle_detected = True
          
    else:
        obstacle_detected = False

         
      

def main():
    global cmd_vel_pub
    global rate
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
    
    # Boucle pour maintenir le node ROS en cours d'exécution.
    rospy.spin()


if __name__ == '__main__':
    try : 
         main()
    except rospy.ROSInterruptException:
        pass

