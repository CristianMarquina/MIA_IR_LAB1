#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import smach
import math
import time

# --- VARIABLES GLOBALES ---
pub = None 
current_yaw = 0.0 

# --- CALLBACKS ---
def odom_callback(msg):
    global current_yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_yaw = yaw

def monitor_laser(msg, userdata):
    userdata.scan = msg
    userdata.data_available = True

# --- ESTADO 1: BARRIDO ---
class GoStraight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wall_detected', 'aborted'], 
                             input_keys=['scan', 'data_available'])

    def execute(self, userdata):
        global pub
        rospy.loginfo("\n>>> [STATE] BARRIDO (Grid Aligned)")
        rate = rospy.Rate(10)
        move = Twist()
        
        while not rospy.is_shutdown():
            if userdata.data_available:
                # Detección de pared simple
                ranges = userdata.scan.ranges
                front_cone = ranges[0:10] + ranges[-10:]
                front_cone = [x if x != float('inf') else 10.0 for x in front_cone]
                min_front = min(front_cone)
                
                if min_front < 0.6:
                    rospy.logwarn("!!! WALL DETECTED (%.2fm) !!!", min_front)
                    move.linear.x = 0.0
                    pub.publish(move)
                    return 'wall_detected'
                
                # CORRECCIÓN DE RUMBO:
                # Aquí podrías añadir lógica para mantener el ángulo recto mientras avanza,
                # pero por simplicidad solo avanzamos.
                move.linear.x = 0.3
                pub.publish(move)
            rate.sleep()
        return 'aborted'

# --- ESTADO 2: GIRO ABSOLUTO (GRID SNAP) ---
class UTurn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_complete', 'aborted'], 
                             input_keys=['scan', 'turn_direction'],
                             output_keys=['turn_direction'])

    def execute(self, userdata):
        global pub
        rospy.loginfo("\n>>> [STATE] U-TURN (SNAP TO GRID)")
        direction = userdata.turn_direction
        
        # Verificar Espacio (Seguridad)
        ranges = userdata.scan.ranges
        left_dist = ranges[90] if ranges[90] != float('inf') else 10.0
        right_dist = ranges[270] if ranges[270] != float('inf') else 10.0
        
        if direction == 'RIGHT' and right_dist < 0.8:
            direction = 'LEFT'
        elif direction == 'LEFT' and left_dist < 0.8:
            direction = 'RIGHT'

        # 1. PRIMER GIRO: ALINEAR AL SIGUIENTE CARDINAL
        self.rotate_absolute_cardinal(direction)
        
        # 2. AVANZAR
        self.move_forward_time(1.0)
        
        # 3. SEGUNDO GIRO: ALINEAR AL SIGUIENTE CARDINAL
        self.rotate_absolute_cardinal(direction)
        
        # Toggle Dirección
        if direction == 'RIGHT':
            userdata.turn_direction = 'LEFT'
        else:
            userdata.turn_direction = 'RIGHT'
            
        return 'turn_complete'

    # --- LA FUNCIÓN QUE PIDES: ALINEACIÓN PERFECTA ---
    def rotate_absolute_cardinal(self, direction):
        global current_yaw, pub
        
        # 1. Definimos los 4 Puntos Cardinales exactos (Radianes)
        # 0 (Este), 1.57 (Norte), 3.14 (Oeste), -1.57 (Sur)
        CARDINALS = [0.0, math.pi/2, math.pi, -math.pi/2]
        
        # 2. Calcular dónde deberíamos caer
        # Estimamos dónde caeríamos si sumamos 90 grados al ángulo actual
        if direction == 'LEFT':
            estimated_target = current_yaw + (math.pi / 2)
        else:
            estimated_target = current_yaw - (math.pi / 2)
            
        # 3. TRUCO MATEMÁTICO (SNAP TO GRID)
        # Buscamos en la lista CARDINALS cuál es el número más cercano a mi estimado.
        # Esto elimina cualquier error de 2 o 3 grados que traiga de antes.
        
        # Primero normalizamos el estimado entre -PI y PI
        while estimated_target > math.pi: estimated_target -= 2*math.pi
        while estimated_target < -math.pi: estimated_target += 2*math.pi
        
        # Buscamos el más cercano
        target_yaw = min(CARDINALS, key=lambda x: abs(x - estimated_target))
        
        # Caso especial: PI y -PI son el mismo lugar (180 grados)
        # Si estamos cerca de 3.14 o -3.14, aseguramos la convergencia
        if abs(target_yaw - math.pi) < 0.1 or abs(target_yaw - (-math.pi)) < 0.1:
            # Preferimos usar el signo que esté más cerca matemáticamente para evitar vueltas largas
            if current_yaw > 0: target_yaw = math.pi
            else: target_yaw = -math.pi

        rospy.loginfo("   -> Grid Snap: Current=%.2f, Target=%.2f (Cardinals)", current_yaw, target_yaw)
        
        # 4. EJECUTAR EL GIRO CON PID
        self.run_pid_loop(target_yaw)

    def run_pid_loop(self, target_yaw):
        global pub, current_yaw
        
        move = Twist()
        rate = rospy.Rate(50)
        
        # PID Constants
        Kp = 1.8
        Ki = 0.05
        Kd = 0.6
        
        prev_error = 0.0
        integral = 0.0
        TOLERANCE = math.radians(0.5) # 0.5 grados de precisión
        
        while not rospy.is_shutdown():
            # Error Calculation
            error = target_yaw - current_yaw
            while error > math.pi: error -= 2*math.pi
            while error < -math.pi: error += 2*math.pi
            
            if abs(error) < TOLERANCE:
                rospy.loginfo("      Aligned! Error: %.4f", error)
                break
            
            # PID Math
            integral += error
            integral = max(min(integral, 0.5), -0.5) # Anti-windup
            derivative = error - prev_error
            prev_error = error
            
            speed = (Kp * error) + (Ki * integral) + (Kd * derivative)
            speed = max(min(speed, 0.8), -0.8) # Limit speed
            
            move.angular.z = speed
            pub.publish(move)
            rate.sleep()
            
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

# --- MAIN ---
if __name__ == '__main__':
    rospy.init_node('zigzag_grid_snap')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
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

    rospy.loginfo("Iniciando ZigZag con ALINEACIÓN ABSOLUTA (Grid Snap)...")
    sm.execute()