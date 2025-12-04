#!/usr/bin/env python

import rospy
import cv2 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# --- CALLBACKS ---

def camera_callback(data):
    """
    Callback function for the camera. 
    It receives the image message, converts it to OpenCV format, and displays it.
    """
    try:
        # Convert the ROS Image message to a BGR image (OpenCV standard)
        # "bgr8" means Blue-Green-Red, 8 bits per channel
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Display the image in a window
        cv2.imshow("Turtlebot3 Camera Viewer", cv_image)
        
        # Determine the wait time (1ms) so the window can refresh
        cv2.waitKey(1)
        
    except CvBridgeError as e:
        rospy.logerr("Camera conversion failed: %s", e)

def laser_callback(msg):
    """
    Callback function for the LaserScan.
    Even if we perform a blind maneuver, it is good practice to keep the sensor active.
    """
    # Just for debugging, we could print data here, but we keep it silent 
    # to avoid cluttering the terminal.
    pass

# --- MAIN PROGRAM ---

if __name__ == '__main__':
    # 1. Initialize the ROS Node
    rospy.init_node('behavior_1_blind_patrol')
    
    # 2. Setup Publishers and Subscribers
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Subscribe to sensors
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.Subscriber('/camera/rgb/image_raw', Image, camera_callback)
    
    # 3. Control Loop Configuration
    rate = rospy.Rate(10) # Loop at 10Hz (10 times per second)
    move = Twist()
    
    rospy.loginfo("STARTING BEHAVIOR 1: Blind Patrol (Rotate & Advance sequence)")
    rospy.loginfo("Press Ctrl+C to stop the robot.")

    # Wait a bit for connections to establish
    rospy.sleep(1)

    while not rospy.is_shutdown():
        
        # --- PHASE 1: ROTATION (2 Seconds) ---
        rospy.loginfo(">> PHASE: Rotating (Scanning area)...")
        
        start_time = rospy.get_time()
        
        # Loop for 2 seconds
        while (rospy.get_time() - start_time) < 2.0:
            move.linear.x = 0.0
            move.angular.z = 0.5  
            
            pub.publish(move)
            rate.sleep()
            
        # --- PHASE 2: MOVE FORWARD (2 Seconds) ---
        rospy.loginfo(">> PHASE: Advancing (Moving forward)...")
        
        start_time = rospy.get_time()
        
        # Loop for 2 seconds
        while (rospy.get_time() - start_time) < 2.0:
            move.linear.x = 0.2   # Move Forward (m/s)
            move.angular.z = 0.0
            
            pub.publish(move)
            rate.sleep()
            
        # The main while loop repeats this sequence indefinitely.