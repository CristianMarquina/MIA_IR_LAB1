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
# --- NEW: ODOMETRY MONITOR (JUST FOR PRINTING) ---
def odom_callback(msg):
    """
    This function runs automatically when the robot moves.
    It DOES NOT control the robot. It only PRINTS the angle.
    """
    global current_yaw
    # 1. Get orientation in Quaternions (x,y,z,w)
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    
    # 2. Convert to Euler Angles (Roll, Pitch, Yaw)
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    
    # 3. Convert Radians to Degrees for humans
    yaw_degrees = math.degrees(yaw)
    roll_degrees = math.degrees(roll)
    pitch_degrees = math.degrees(pitch)
    
    current_yaw = yaw
    # 4. Print it every 0.5 seconds
    rospy.loginfo_throttle(0.5, ">>> [MONITOR] Current Angle (Yaw): %.2f ", yaw)
    rospy.loginfo_throttle(0.5, ">>> [MONITOR] Current Angle (Yaw): %.2f degrees", yaw_degrees)
# --- MONITOR ---
def monitor_laser(msg, userdata):
    userdata.scan = msg
    userdata.data_available = True

# --- STATE 1:  ---
class GoStraight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wall_detected', 'aborted'], 
                             input_keys=['scan', 'data_available'])

    def execute(self, userdata):
        global pub
        rospy.loginfo("\n>>> [STATE] BARRIDO (SWEEPING)")
        rate = rospy.Rate(10)
        move = Twist()
        
        while not rospy.is_shutdown():
            if userdata.data_available:
                ranges = userdata.scan.ranges
                # Cono frontal
                front_cone = ranges[0:10] + ranges[-10:]
                front_cone = [x if x != float('inf') else 10.0 for x in front_cone]
                min_front = min(front_cone)
                
                # Telemetría ligera
                rospy.loginfo_throttle(2, "[SWEEP] Wall distance: %.2fm", min_front)
                
                if min_front < 0.6:
                    rospy.logwarn("!!! WALL DETECTED (%.2fm). Stopping !!!", min_front)
                    move.linear.x = 0.0
                    pub.publish(move)
                    return 'wall_detected'
                
                move.linear.x = 0.3
                pub.publish(move)
            rate.sleep()
        return 'aborted'

# --- STATE 2: ---
class UTurn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_complete', 'aborted'], 
                             input_keys=['scan', 'turn_direction'],
                             output_keys=['turn_direction'])

    def execute(self, userdata):
        global pub
        rospy.loginfo("\n>>> [STATE] PRECISION U-TURN")
        
        direction = userdata.turn_direction
        
        #VERIFICACIÓN DE ESPACIO (Seguridad) 
        ranges = userdata.scan.ranges
        left_dist = ranges[90] if ranges[90] != float('inf') else 10.0
        right_dist = ranges[270] if ranges[270] != float('inf') else 10.0
        
        # Si está bloqueado, invertimos
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
    
      
        """ # --- FASE 1: PRIMER GIRO DE 90 GRADOS ---
        self.perform_precision_turn(direction)
        
        # --- FASE 2: DESPLAZAMIENTO LATERAL (CARRIL) ---
        # Ajusta este tiempo para separar más o menos las líneas de barrido
        self.perform_lane_shift(duration=1.5)
        
        # --- FASE 3: SEGUNDO GIRO DE 90 GRADOS ---
        self.perform_precision_turn(direction)
        
        # --- ACTUALIZAR MEMORIA ---
        if direction == 'RIGHT':
            userdata.turn_direction = 'LEFT'
        else:
            userdata.turn_direction = 'RIGHT'
            
        return 'turn_complete'"""


    def rotate_precise(self, target_angle_rad, direction):
        global current_yaw, pub
        
        initial_yaw = current_yaw
        
        # Calculamos cuál es el ángulo final deseado
        if direction == 'LEFT':
            target_yaw = initial_yaw + target_angle_rad
        else:
            target_yaw = initial_yaw - target_angle_rad
            
        rospy.loginfo("   -> Turning Precise: Start=%.2f, Target=%.2f", initial_yaw, target_yaw)
        
        move = Twist()
        rate = rospy.Rate(50) 
        
        # Bucle de Control IN P, control proprocional fue lo primero que hice
        """while not rospy.is_shutdown():
            # 1. Calcular el ERROR (Cuánto me falta)
            diff = target_yaw - current_yaw
            
            # Matemáticas para corregir el salto de +/- 180 grados
            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi
            
            # 2. error de salida y lo que permitimos es 0.05 radianes (~3 grados)
            if abs(diff) < 0.05:
                break
            
            # 3. Velocidad Proporcional el control proporcional
            speed = max(0.1, min(0.5, abs(diff)))
            
            if diff > 0:
                move.angular.z = speed  
            else:
                move.angular.z = -speed 
                
            pub.publish(move)
            rate.sleep()
        """ 


        
        Kp = 1.5
        Ki = 0.02
        Kd = 0.5
        
        prev_error = 0.0
        integral = 0.0
        
        # Tolerancia muy fina (0.2 grados)
        TOLERANCE = math.radians(0.2)
        # bucle de control PID para llegar al angulo exacto
        while not rospy.is_shutdown():
            # 1. Calcular Error (P)
            error = target_yaw - current_yaw
            while error > math.pi: error -= 2*math.pi
            while error < -math.pi: error += 2*math.pi
            
            # 2. Verificar salida
            if abs(error) < TOLERANCE:
                rospy.loginfo("      Target reached! Error: %.5f rad", error)
                break
            
            # 3. Calcular Integral (I) - Acumula error en el tiempo
            integral += error
            # Anti-Windup: Evitamos que la integral crezca infinito si se atasca
            integral = max(min(integral, 1.0), -1.0)
            
            # 4. Calcular Derivada (D) - Velocidad de cambio del error
            derivative = error - prev_error
            prev_error = error
            
            # 5. LA FÓRMULA MAESTRA PID
            output_speed = (Kp * error) + (Ki * integral) + (Kd * derivative)
            
            # 6. Limitar velocidad máxima (Seguridad)
            # No queremos que gire a más de 0.8 rad/s
            output_speed = max(min(output_speed, 0.8), -0.8)
            
            # NOTA: Con PID, no necesitamos forzar una velocidad mínima (0.02),
            # porque la parte INTEGRAL se encargará de empujar si se queda corto.
            
            move.angular.z = output_speed
            pub.publish(move)
            rate.sleep()
        # Parada Final
        move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(0.5)

    def move_forward_time(self, duration):
        move = Twist()
        move.linear.x = 0.2
        t0 = rospy.get_time()
        while (rospy.get_time() - t0) < duration:
            pub.publish(move)
            rospy.sleep(0.01)
        move.linear.x = 0.0
        pub.publish(move)
        rospy.sleep(0.2)
    # --- FUNCIÓN DE GIRO MATEMÁTICO ---
    def perform_precision_turn(self, direction):
        move = Twist()
        
        # CONFIGURACIÓN FÍSICA
        SPEED = 0.5  # Velocidad de giro (rad/s)
        TARGET_ANGLE = math.pi / 2  # 90 grados exactos (1.57 rad)
        
        # FACTOR DE CALIBRACIÓN (TOCA ESTO SI NO QUEDA PARALELO)
        # 1.0 = Matemáticamente perfecto
        # > 1.0 = Gira más tiempo (si se queda corto)
        # < 1.0 = Gira menos tiempo (si se pasa)
        CALIBRATION = 1.0  
        
        # Cálculo del tiempo exacto: T = Angulo / Velocidad
        turn_duration = (TARGET_ANGLE / SPEED) * CALIBRATION
        
        rospy.loginfo("   -> Turning 90 deg (Time: %.3fs)", turn_duration)

        if direction == 'LEFT':
            move.angular.z = SPEED
        else:
            move.angular.z = -SPEED
            
        t0 = rospy.get_time()
        while (rospy.get_time() - t0) < turn_duration:
            pub.publish(move)
            rospy.sleep(0.01) # Alta frecuencia para precisión
        
        # Parada en seco
        move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(0.5) # Esperar a que la física se estabilice
        
    def perform_lane_shift(self, duration):
        rospy.loginfo("   -> Shifting Lane (Time: %.1fs)", duration)
        move = Twist()
        move.linear.x = 0.2
        t0 = rospy.get_time()
        while (rospy.get_time() - t0) < duration:
            pub.publish(move)
            rospy.sleep(0.01)
        move.linear.x = 0.0
        pub.publish(move)
        rospy.sleep(0.2)

# --- MAIN ---
if __name__ == '__main__':
    rospy.init_node('behavior_zigzag_calibrated')
    # getting yaw
    rospy.Subscriber('/odom', Odometry, odom_callback)
    # Global Publisher for velocity commands
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