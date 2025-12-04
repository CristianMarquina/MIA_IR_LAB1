#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import smach
import numpy as np

# Global Publisher
pub = None 

# --- MONITOR CALLBACK ---
def monitor_laser(msg, userdata):
    userdata.scan = msg
    userdata.data_available = True

# --- STATE 1: FIND WALL ---
class FindWall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wall_found', 'aborted'], 
                             input_keys=['scan', 'data_available'])

    def execute(self, userdata):
        global pub
        rospy.loginfo(">>> STATE TRANSITION: Entering [FIND WALL] State")
        rate = rospy.Rate(10)
        move = Twist()
        
        while not rospy.is_shutdown():
            if userdata.data_available:
                # 1. Get raw data
                front_dist = userdata.scan.ranges[0]
                if front_dist == float('inf'): front_dist = 10.0
                
                # 2. Logic & Visualization
                if front_dist < 0.5:
                    rospy.logwarn("[-] WALL DETECTED (Distance: %.2fm). Stopping and Switching State.", front_dist)
                    move.linear.x = 0.0
                    pub.publish(move)
                    return 'wall_found'
                
                # Detailed logging every 1 second
                rospy.loginfo_throttle(1, 
                    "[STATE: FIND WALL] Looking for wall... | Front Scan: %.2fm | Speed: 0.2 m/s", 
                    front_dist)
                
                move.linear.x = 0.2
                pub.publish(move)
            rate.sleep()
        return 'aborted'

# --- STATE 2: FOLLOW WALL (P-CONTROLLER) ---
class FollowWall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['mission_completed', 'aborted'], 
                             input_keys=['scan', 'data_available'])

    def execute(self, userdata):
        global pub
        rospy.loginfo(">>> STATE TRANSITION: Entering [FOLLOW WALL] State")
        rate = rospy.Rate(10)
        move = Twist()
        
        # --- CONTROLLER CONFIGURATION ---
        TARGET_DIST = 0.40  # Desired distance (meters)
        KP = 2.5            # Proportional Gain (Reaction strength)
        
        while not rospy.is_shutdown():
            if userdata.data_available:
                ranges = userdata.scan.ranges
                
                # 1. RAW SENSOR DATA
                front = ranges[0]
                # Right sector average (Indices 260-280)
                right_sector = ranges[260:280]
                right_sector = [x if x != float('inf') else 10.0 for x in right_sector]
                
                if len(right_sector) > 0:
                    right_dist = min(right_sector)
                else:
                    right_dist = 10.0

                # 2. LOGIC & DECISION MAKING
                action_description = ""
                
                # --- EMERGENCY CASE (Corner) ---
                if front < 0.5:
                    action_description = "EMERGENCY: Wall Ahead! Spinning Left"
                    move.linear.x = 0.05
                    move.angular.z = 0.8 # Strong turn
                    
                else:
                    # --- P-CONTROLLER ---
                    # Error Calculation: Positive error means "Too Close" (Need to turn Left)
                    error = TARGET_DIST - right_dist
                    
                    # Calculate Correction (Angular Velocity)
                    rotation_z = error * KP
                    
                    # Limit the rotation (Saturation) to avoid spinning out of control
                    rotation_z = max(min(rotation_z, 1.0), -1.0)
                    
                    # Determine Action Description for the Log
                    if abs(error) < 0.05:
                        action_description = "Stable (Good Distance)"
                    elif error > 0:
                        action_description = "Too Close -> Correcting LEFT"
                    else:
                        action_description = "Too Far -> Correcting RIGHT"

                    move.linear.x = 0.15
                    move.angular.z = rotation_z
                
                # 3. DETAILED TELEMETRY PRINT (Every 0.5 seconds)
                # We format it as a table-like structure for readability
                log_msg = (
                    f"\n--- [STATE: FOLLOW WALL] ---\n"
                    f"  SENSORS  | Front: {front:.2f}m | Right: {right_dist:.2f}m (Target: {TARGET_DIST}m)\n"
                    f"  LOGIC    | Error: {TARGET_DIST - right_dist:.2f} | Action: {action_description}\n"
                    f"  COMMANDS | Linear X: {move.linear.x:.2f} m/s | Angular Z: {move.angular.z:.2f} rad/s"
                )
                rospy.loginfo_throttle(0.5, log_msg)

                # 4. SEND COMMAND
                pub.publish(move)
                
            rate.sleep()
            
        return 'aborted'

# --- MAIN ---
if __name__ == '__main__':
    rospy.init_node('behavior_wall_follower_verbose')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sm = smach.StateMachine(outcomes=['success', 'aborted'])
    sm.userdata.scan = LaserScan()
    sm.userdata.data_available = False
    
    rospy.Subscriber('/scan', LaserScan, monitor_laser, callback_args=sm.userdata)
    
    with sm:
        smach.StateMachine.add('FIND_WALL', FindWall(), 
                               transitions={'wall_found':'FOLLOW_WALL', 'aborted':'aborted'})
        smach.StateMachine.add('FOLLOW_WALL', FollowWall(), 
                               transitions={'mission_completed':'success', 'aborted':'aborted'})

    rospy.loginfo("System Initialized. Starting State Machine...")
    sm.execute()