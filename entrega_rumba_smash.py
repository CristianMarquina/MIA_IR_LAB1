import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry  # <--- NEW: To read position
from tf.transformations import euler_from_quaternion # <--- NEW: To do math
import smach
import math
import time

# Global Publisher
pub = None 
current_yaw = 0.0

def odom_callback(msg):
    """
    This function runs automatically when the robot moves.
    It DOES NOT control the robot. It only PRINTS the angle.
    """
    global current_yaw
    #Get orientation in Quaternions (x,y,z,w)
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    
    # Convert to Euler Angles (Roll, Pitch, Yaw)
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    
    # Convert Radians to Degrees for humans
    yaw_degrees = math.degrees(yaw)
    roll_degrees = math.degrees(roll)
    pitch_degrees = math.degrees(pitch)
    
    current_yaw = yaw
    # Print it every 0.5 seconds
    rospy.loginfo_throttle(0.5, ">>> [MONITOR] Current Angle (Yaw): %.2f degrees", yaw_degrees)


def monitor_laser(msg, userdata):
    """
    Callback function to update the shared Blackboard (userdata).
    
    Args:
        msg (LaserScan): The raw data coming from the /scan topic.
        userdata (list): The SMACH userdata object to store the scan.
    
    Returns:
        None. It updates 'scan' and sets 'data_available' flag to True.
    """
    userdata.scan = msg
    userdata.data_available = True


class GoStraight(smach.State):
    """
    A SMACH State that drives the robot forward while actively maintaining 
    a fixed heading (North, South, East, or West).
    
    Behaviors:
    1. Snaps the target heading to the nearest Cardinal direction upon entry.
    2. Uses a P-Controller to correct angular deviations while moving.
    3. Stops and exits if an obstacle is detected within 0.6m.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['wall_detected', 'aborted'], 
                             input_keys=['scan', 'data_available'])

    def execute(self, userdata):
        global pub, current_yaw
        rospy.loginfo("\n>>> [STATE] GoStraight ")
        rate = rospy.Rate(10)
        move = Twist()
        
        CARDINALS = [0.0, math.pi/2, math.pi, -math.pi/2]
        
        normalized_yaw = current_yaw
        
        target_angle = min(CARDINALS, key=lambda x: abs(x - normalized_yaw))
        
        if abs(normalized_yaw) > 3.0:
            if normalized_yaw > 0: target_angle = math.pi
            else: target_angle = -math.pi

        rospy.loginfo("    -> Rumbo fijado: %.2f rad (%.2f deg)", target_angle, math.degrees(target_angle))
        
        # Proportional Gain
        KP_CORRECTION = 1.0

        while not rospy.is_shutdown():
            if userdata.data_available:
                #Detect walls
                ranges = userdata.scan.ranges
                front_cone = ranges[0:10] + ranges[-10:]
                front_cone = [x if x != float('inf') else 10.0 for x in front_cone]
                min_front = min(front_cone)
                
                # Telemetry
                rospy.loginfo_throttle(1, "[SWEEP] Wall: %.2fm | Angle Err: %.2f deg", 
                                    min_front, math.degrees(target_angle - current_yaw))
                
                if min_front < 0.9:
                    rospy.logwarn("!WALL DETECTED (%.2fm). Stopping !", min_front)
                    move.linear.x = 0.0
                    move.angular.z = 0.0
                    pub.publish(move)
                    return 'wall_detected'
                
                #error yaw
                error_yaw = target_angle - current_yaw

                # Normalization to [-pi, pi]
                while error_yaw > math.pi: error_yaw -= 2*math.pi
                while error_yaw < -math.pi: error_yaw += 2*math.pi
                
                # Apply correction
                correction_z = error_yaw * KP_CORRECTION
                
                # Limit correction to avoid abrupt "S" turns
                correction_z = max(min(correction_z, 0.3), -0.3)

                # Set velocities
                move.linear.x = 0.5      
                move.angular.z = correction_z # 
                
                pub.publish(move)
            rate.sleep()


class UTurn(smach.State):
    """
    A SMACH State that executes a precise U-Turn maneuver to switch search lanes.
    
    This state performs a sequence of three actions:
    1. First 90-degree turn (PID controlled).
    2. Lateral displacement (Lane shift).
    3. Second 90-degree turn (PID controlled).
    
    It also toggles the 'turn_direction' in userdata to ensure the robot 
    alternates between Left and Right turns (Zig-Zag pattern).
    """
    def __init__(self):
        """
        Initializes the state.
        
        Input Keys:
            scan (LaserScan): To check for side obstacles.
            turn_direction (str): 'LEFT' or 'RIGHT', decides the turn direction.
            
        Output Keys:
            turn_direction (str): Updates the direction for the next turn.
        """
        smach.State.__init__(self, outcomes=['turn_complete', 'aborted'], 
                             input_keys=['scan', 'turn_direction'],
                             output_keys=['turn_direction'])

    def execute(self, userdata):
        """
        Main execution logic for the U-Turn maneuver.
        
        Steps:
        1. Retrieve desired direction from userdata.
        2. Safety Check: Verify if the desired side is blocked by a wall.
           If blocked, invert the direction to avoid collision.
        3. Execute the 3-step maneuver (Turn -> Advance -> Turn).
        4. Update 'turn_direction' for the next iteration.
        """
        global pub
        rospy.loginfo("\n[STATE] PRECISION U-TURN")
        
        direction = userdata.turn_direction
        
        # Verify if side is blocked
        ranges = userdata.scan.ranges
        left_dist = ranges[90] if ranges[90] != float('inf') else 10.0
        right_dist = ranges[270] if ranges[270] != float('inf') else 10.0
        
        # If blocked, invert direction
        if direction == 'RIGHT' and right_dist < 0.8:
            rospy.logwarn("   [BLOCKED RIGHT] Switching to LEFT")
            direction = 'LEFT'
        elif direction == 'LEFT' and left_dist < 0.8:
            rospy.logwarn("   [BLOCKED LEFT] Switching to RIGHT")
            direction = 'RIGHT'
            
        rospy.loginfo("   Executing maneuver to the [%s]", direction)
        target_angle_rad = math.pi / 2  
        self.rotate_precise(target_angle_rad, direction)
        
        self.move_forward_time(1.0)
        
        self.rotate_precise(target_angle_rad, direction)
        
        if direction == 'RIGHT':
            userdata.turn_direction = 'LEFT'
        else:
            userdata.turn_direction = 'RIGHT'
            
        return 'turn_complete'
    
    def rotate_precise(self, target_angle_rad, direction):
        """
        Executes a precise rotation using a PID Controller based on Odometry (Yaw).
        This method ensures the robot rotates the exact amount regardless of friction or battery level.

        Args:
            target_angle_rad (float): The amount to rotate in radians (relative).
            direction (str): 'LEFT' or 'RIGHT'.
        """
        global current_yaw, pub
        
        initial_yaw = current_yaw
        
        # Calculate the desired final angle
        if direction == 'LEFT':
            target_yaw = initial_yaw + target_angle_rad
        else:
            target_yaw = initial_yaw - target_angle_rad
            
        rospy.loginfo("   -> Turning Precise: Start=%.2f, Target=%.2f", initial_yaw, target_yaw)
        
        move = Twist()
        rate = rospy.Rate(50) 

        # PID CONTROLLER GAINS      
        # Proportional gain
        Kp = 1.5
        # Integral gain
        Ki = 0.02
        # Derivative gain
        Kd = 0.5
        
        prev_error = 0.0
        integral = 0.0
        
        # Very fine tolerance (0.2 degrees)
        TOLERANCE = math.radians(0.2)
        # PID control loop to reach the exact angle
        while not rospy.is_shutdown():
            # Calculate Error (P)
            error = target_yaw - current_yaw
            while error > math.pi: error -= 2*math.pi
            while error < -math.pi: error += 2*math.pi
            
            # 2. Check exit condition
            if abs(error) < TOLERANCE:
                rospy.loginfo("      Target reached! Error: %.5f rad", error)
                break
            
            # 3. Calculate Integral (I) - Accumulates error over time
            integral += error
            # Anti-Windup: Prevent integral from growing infinitely if stuck
            integral = max(min(integral, 1.0), -1.0)
            
            # 4. Calculate Derivative (D) - Rate of change of error
            derivative = error - prev_error
            prev_error = error
            
            # 5. THE MASTER FORMULA PID
            output_speed = (Kp * error) + (Ki * integral) + (Kd * derivative)
            
            # 6. Limit maximum speed (Safety)
            # We don't want it to turn faster than 0.8 rad/s
            output_speed = max(min(output_speed, 0.8), -0.8)
            
            # NOTE: With PID, we don't need to enforce a minimum speed (0.02),
            # because the INTEGRAL part will push if it falls short.
            
            move.angular.z = output_speed
            pub.publish(move)
            rate.sleep()
        # Final Stop
        move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(0.5)

    def move_forward_time(self, duration):
        """
        Moves the robot forward blindly for a specific duration.
        Used for small displacements where high precision is not critical.
        
        Args:
            duration (float): Time in seconds to move forward.
        """
        move = Twist()
        move.linear.x = 0.2
        t0 = rospy.get_time()
        while (rospy.get_time() - t0) < duration:
            pub.publish(move)
            rospy.sleep(0.01)
        move.linear.x = 0.0
        pub.publish(move)
        rospy.sleep(0.2)


if __name__ == '__main__':
    """
    Main execution block.
    Initializes the ROS node, hardware interfaces, and the State Machine topology.
    """
    rospy.init_node('behavior_zigzag_calibrated')
    # Odometer Subscriber for angle tracking
    rospy.Subscriber('/odom', Odometry, odom_callback)
    # Velocity Publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sm = smach.StateMachine(outcomes=['success', 'aborted'])
    
    sm.userdata.scan = LaserScan()
    sm.userdata.data_available = False
    sm.userdata.turn_direction = 'RIGHT' 
    
    rospy.Subscriber('/scan', LaserScan, monitor_laser, callback_args=sm.userdata)
    
    with sm:
        smach.StateMachine.add('GO_STRAIGHT', GoStraight(), 
                               transitions={'wall_detected':'U_TURN', 'aborted':'aborted'})
        smach.StateMachine.add('U_TURN', UTurn(), 
                               transitions={'turn_complete':'GO_STRAIGHT', 'aborted':'aborted'})

    rospy.loginfo("Starting Calibrated Zig-Zag...")
    sm.execute()