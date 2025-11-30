#!/usr/bin/env python

import rospy
import numpy as np 
import cv2 #opencv-python (cv_bridge)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#To work with images
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


#we create an empty global variable
laser=None
frame=None
encoding=None


def img_callback(data):
   global frame,encoding
   
   bridge=CvBridge()
   #imgmsg_to_cv2 allows converting ROS messages into image matrices used by OpenCV
   # (numpy.ndarray)
   frame=bridge.imgmsg_to_cv2(data)
   encoding=data.encoding
   
   #IMPORTANT FOR STUDENT ....
   # COMMENT THE NEXT LINES WHEN YOU DO NOT NEED TO SEE THE IMAGE ANY MORE.... 
   #the image is shown
   cv2.imshow('Image viewer',frame)
   cv2.waitKey(1)
   
   #we can change the encoding..., by defautl the message Image is in mode RGB
   #by default this encoding is preserved
   #but it is possible to change it to BGR which is the encoding used by OpenCV
   frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
   encoding='BGR'
   
   


def scan_callback(msg):
    global range_ahead
    global laser

    laser=msg



rospy.init_node ('movedor_robot')
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
scan_sub=rospy.Subscriber('/scan',LaserScan,scan_callback)

move=Twist()




#### IMPORTANT FOR THE STUDENT 

#if you do not wish to work with images comment the next line...
img_sub=rospy.Subscriber('/camera/rgb/image_raw',Image,img_callback)



input ("we are going to see the laser readings, press to continue")


print ("laser readings",laser.ranges)
print ("type of variable ",type(laser))
print ("type of variable ",type(laser.ranges))

#we can see the characteristics of our readings....

print ("angle min %f",laser.angle_min) #0
print ("angle max %f",laser.angle_max) #2*pi
print ("angle_increment %f",laser.angle_increment)  # angular distance between measurements [rad]
print ("minimun distance that the sensor can detect ",laser.range_min)
print ("maximum distance that the sensor can detect ",laser.range_max)

input ("press to see the readings and the angle ")

#first we should locate where the cero starts in our robot ....
#zero is front left, it increases until front rignt 
#you can check this by placing something close to the robot...
#we can see the values of the laser together with the angle and index of the vector (tuple)
for i in range (0,len(laser.ranges),1):
   currentangle=laser.angle_min+i*laser.angle_increment
   print ("i %d current angle %f distance %f",i,currentangle,laser.ranges[i])


input ("press to continue and see the information about the image aquired from the robot ")

print ("image encoding ", encoding)
(h,w,c)=frame.shape
print ("height: ",h," width: ",w," channels: ",c)


input ("press to start moving the robot ")
#The number withing brackets represent the control frequency (in this example is 4Hz)
rate=rospy.Rate(4)    

while not rospy.is_shutdown(): #This is the control cycle, it keeps moving the robot 
#until Ctrl+C is pressed
   #The next value establishes the linear velocity of the robot	
   move.linear.x=0.0
   #The next value fixes the angular velocity of the robot (rad/sec)
   move.angular.z=0.5
   
   #the next line publishes the velocities (so that the robot moves)
   pub.publish(move)
   
   #The next line makes the program wait so that the control frequency is met.
   rate.sleep()

        



