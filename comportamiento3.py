#!/usr/bin/env python
#Este es un ejemplo con dos estados
# En este programa se muestra el de userdata como pizarra de datos
#Este programa define un callback que hace de monitor del topic, la idea es que si recibe un mensaje a traves del topic 
#cambia el valor de una varible global que se detecta desde el estado y sale.
#esto simula por ejemplo la situacion en la que si se detecta un valor cercano desde callback sensor (o fuerza repulsiva), se activa el estado evitar obstaculo


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
import sys
import math
import numpy as np
import smach


#Hay una parte inicial del codigo para comprobar sentido de las medidas del laser
#se dividen las lecturas en 8 sectores
#se calcula la fuerza repulsiva total (modulo y signo) como resultado de calcular las fuerzas repulsivase en el primer y el ultimo sector (parte frontal del robot)


def monitor_laser(msg, userdata):
   """ Monitor function for LaserScan messages.
       If an obstacle is detected within 0.6m in front of the robot,
       it sets a stop_flag in userdata to signal the state machine.
   """
    
   #read front distance from laser scan
   distancia = msg.ranges[0]
   if distancia == float('inf'):
      distancia = 10.0
   rospy.loginfo("MONITOR: (%.2fm)", distancia)
   # if obstacle is closer than 0.6m, set stop_flag in userdata
   if distancia < 0.6:
      if userdata.stop_flag == 0:
         rospy.logwarn(">>> MONITOR: Obstacle detected (%.2fm)! ", distancia)
         userdata.stop_flag = 1

class Muevete(smach.State):
   def __init__(self):
        """
        Move state: advances while stop_flag in userdata is 0
        """
        # input_keys=['stop_flag'] -> Pedimos permiso para LEER esta variable
        smach.State.__init__(self, outcomes=['exito','fallo'], input_keys=['stop_flag'])
      
   def execute(self, userdata):
      """
      execute method for Move state, moves forward until stop_flag in userdata changes
      """
      global pub
      rospy.loginfo("STATE MUEVETE: Reading Blackboard... Flag is %d", userdata.stop_flag)
      
      rate = rospy.Rate(10)
      move = Twist()
      
      # Move forward while stop_flag is 0
      while not rospy.is_shutdown() and userdata.stop_flag == 0:
            move.linear.x = 0.2
            move.angular.z = 0.0
            pub.publish(move)
            rate.sleep()
      
      # If we exit the while loop, it means stop_flag changed to 1 (or ROS was shut down)
      rospy.loginfo("STATE MUEVETE: Flag changed to 1. Stopping.")
      move.linear.x = 0.0
      pub.publish(move)
      return 'exito'
      
class EvitaObstaculo(smach.State):
   """
   EvitaObstaculo state: performs a blind turn to avoid obstacle"""
   def __init__(self):
        # output_keys=['stop_flag'] -> Pedimos permiso para ESCRIBIR/MODIFICAR esta variable
        smach.State.__init__(self, outcomes=['exito','fallo'], output_keys=['stop_flag'])
   
   def execute(self, userdata):
      """
      execute method for EvitaObstaculo state, performs a blind turn
      """
      global pub
      rospy.loginfo("STATE EVITA: Turning...")
      
      # Perform blind turn by 2 seconds
      move = Twist()
      move.linear.x = 0.0
      move.angular.z = 0.5 
      
      t_init = rospy.get_time()
      while (rospy.get_time() - t_init) < 2.0:
         pub.publish(move)
         rospy.sleep(0.1)
         
      move.angular.z = 0.0
      pub.publish(move)
      
      # After maneuver, reset stop_flag in userdata to 0
      rospy.loginfo("STATE EVITA: Resetting Blackboard flag to 0.")
      userdata.stop_flag = 0
      
      return 'exito'
      


      
#Modulo principal:
rospy.init_node('arquitectura_navegacion')


#creamos la maquina de estados compuesta
maquina_compuesta=smach.StateMachine(outcomes=['exito','fallo'])
muevete_state=Muevete()
evita_obstaculo_state=EvitaObstaculo()


maquina_compuesta.userdata.stop_flag = 0
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.Subscriber('/scan', LaserScan, monitor_laser, callback_args=maquina_compuesta.userdata)

with maquina_compuesta:
   #agregamos estados a la maquina de estados compuesta
   smach.StateMachine.add('MUEVETE', Muevete(), 
                               transitions={'exito':'EVITAOBSTACULO','fallo':'fallo'})
   smach.StateMachine.add('EVITAOBSTACULO', EvitaObstaculo(), 
                               transitions={'exito':'MUEVETE','fallo':'fallo'})
   
#ejecutamos la maquina de estados compuesta:
maquina_compuesta.execute()

