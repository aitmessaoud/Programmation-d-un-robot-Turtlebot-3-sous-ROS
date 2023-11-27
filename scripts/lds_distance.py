#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  1 23:12:41 2023

@author: turtle
"""

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
 
def stop_robot():
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
def callback(data):
    global cmd_vel_pub
    global l,r
    front_range=[] 
    right_range=[]
    back_range=[]
    left_range=[]
    ranges=data.ranges[0:50] + data.ranges[310:359]
    for rng in ranges : 
        if rng < 3 : 
           front_range.append(rng)
             
        else : 
           front_range.append(3)
    front = min(front_range)
    
    for rng in data.ranges[45:137] : 
        if rng < 3 : 
           left_range.append(rng)
        else : 
           left_range.append(3)
    left = min(left_range)
    
    for rng in data.ranges[233:317] : 
        if rng < 3 : 
          right_range.append(rng)
        else : 
          right_range.append(3)
    right = min(right_range) 
    
    for rng in data.ranges[122 : 238] : 
        if rng < 3 :
           back_range.append(rng)
        else :  
           back_range.append(3)
    back = min(back_range) 

    twist= Twist()
    #obstacles = [front,left,right,back]
    #print("front = ",front,"left = ",left,"right = ",right,"back = ",back) 
    seuil = 0.19
    ang_vel = 1.1
    factor = 0.7
    if front < seuil:
       try: print(l,r)
       except :pass
       rospy.loginfo("obstacle !!!")	
       stop_robot()
       if right < left:
          #turn left :
           l = False 
           try :
               if r == False  : 
                 factor+=0.1                                         # si le robot se met à tourner sans avancer 
                 print("factore increment",factor)                   # alors on augmente le "factor"afin d'augmenter le seuil de la vitesse angulaire
           except : pass                                             # on fait en sorte d'augmenter la vitesse angulaire pour que le robot fuit l'obstacle
           rospy.loginfo("turn left") 
           if front < factor*seuil:          # on définit un autre seuil pour augmenter la vitesse angulaire 
               twist.angular.z = 3*ang_vel   # dans le cas où le robot doit tourner suffisament pour fuir un obstacle 
           else : 
               twist.angular.z = ang_vel
           
       else :
           #turn right
           rospy.loginfo("turn right") 
           r = False 
           try:
             if l == False : 
               factor +=0.1
               print("factore increment",factor)
           except : pass
           if front < factor*seuil:
               twist.angular.z = -3*ang_vel
           else : 
               twist.angular.z = -ang_vel
             
        
           
    else : 
       r = True 
       l = True
       twist.linear.x = 0.2
    cmd_vel_pub.publish(twist)
         


def lds_distance():
    global cmd_vel_pub
    rospy.init_node('lds_distance', anonymous=True)
    
    rospy.Subscriber("scan", LaserScan, callback)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.on_shutdown(stop_robot)
    rospy.spin()

if __name__ == '__main__':
    try:
        lds_distance()
    except rospy.ROSInterruptException:
        pass

