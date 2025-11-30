import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import smach

# SENSOR MONITOR (CALLBACK) 
def scan_callback(msg, userdata):
    userdata.laser = msg
    userdata.laser_available = True

# --- STATE 1: MOVE FORWARD ---
class Move(smach.State):
    def __init__(self):
        # Initialize state with input keys to access shared data
        smach.State.__init__(self, outcomes=['exito', 'fallo'], input_keys=['laser', 'laser_available'])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("STATE: MOVE - Advancing while scanning...")
        rate = rospy.Rate(10)  
        cmd = Twist()
        
        while not rospy.is_shutdown():
            if userdata.laser_available:
                # ACCESS RAW DATA
                raw_ranges = userdata.laser.ranges

                # DATA MANIPULATION (Creating a Field of View)
                # We extract a 20-degree cone from the front:
                front_left = raw_ranges[0:10]
                front_right = raw_ranges[-10:] 
                frontal_cone = front_left + front_right
                
                # DATA CLEANING (Filter 'inf' values)
                # We treat it as a safe distance (10m).
                clean_cone = [x if x != float('inf') else 10.0 for x in frontal_cone]
                
                # DECISION MAKING
                min_distance = min(clean_cone)
                
                # TELEMETRY / DEBUGGING
                rospy.loginfo_throttle(1, "\n--- SENSOR DIAGNOSTICS (MOVE) ---")
                rospy.loginfo_throttle(1, "Sensor Used: %s", "LDS-01 (Laser Distance Sensor)")
                rospy.loginfo_throttle(1, "Raw Data Length: %d rays", len(raw_ranges))
                rospy.loginfo_throttle(1, "Raw Sample (Index 0): %.3f m", raw_ranges[0])
                rospy.loginfo_throttle(1, "Processed Min Distance (Front): %.3f m", min_distance)
                
                # Safety Threshold: 0.6 meters
                if min_distance < 0.6:
                    rospy.logwarn("OBSTACLE DETECTED! Distance: %.2f m. Initiating evasion.", min_distance)
                    
                   
                    cmd.linear.x = 0.0
                    self.pub.publish(cmd)
                    return 'exito' 
            
            
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            rate.sleep()
            
        return 'fallo'

#  AVOID OBSTACLE STATE
class AvoidObstacle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exito', 'fallo'], input_keys=['laser', 'laser_available'])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("STATE: AVOID - Rotating to find clear path...")
        rate = rospy.Rate(10)
        cmd = Twist()
        
        while not rospy.is_shutdown():
            if userdata.laser_available:
                # Reuse the same data processing logic
                ranges = userdata.laser.ranges
                frontal_cone = ranges[0:10] + ranges[-10:]
                clean_cone = [x if x != float('inf') else 10.0 for x in frontal_cone]
                min_distance = min(clean_cone)
                
                rospy.loginfo_throttle(2, "Status: Rotating... Frontal Distance: %.2f m", min_distance)

                # Hysteresis: We wait for more space (1.0m) than the trigger (0.6m) to avoid oscillating
                if min_distance > 1.0:
                    rospy.loginfo("PATH CLEAR (%.2f m)! Resuming navigation.", min_distance)
                    
                    # Stop rotation
                    cmd.angular.z = 0.0
                    self.pub.publish(cmd)
                    return 'exito' # Return to MOVE state
            
            # Rotate left
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 
            self.pub.publish(cmd)
            rate.sleep()
            
        return 'fallo'


if __name__ == '__main__':
    rospy.init_node('navigation_architecture')

    # Create the State Machine
    sm = smach.StateMachine(outcomes=['exito', 'fallo'])
    
    sm.userdata.laser = LaserScan()
    sm.userdata.laser_available = False

    # Subscriber: Connects the sensor topic to the callback
    rospy.Subscriber('/scan', LaserScan, scan_callback, callback_args=sm.userdata)

    with sm:
        smach.StateMachine.add('MOVE', Move(), 
                               transitions={'exito':'AVOID_OBSTACLE', 'fallo':'fallo'})
        
        smach.StateMachine.add('AVOID_OBSTACLE', AvoidObstacle(), 
                               transitions={'exito':'MOVE', 'fallo':'fallo'}) 

    # Execute the State Machine
    rospy.loginfo("Starting Autonomous Navigation Behavior...")
    sm.execute()