#!/usr/bin/env python
#Este es un ejemplo con dos estados
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


class Muevete(smach.State):
   "Move state: advances while estado_debe_ser is 0 (green light)"
   def __init__(self):
        
        smach.State.__init__(self, outcomes=['exito','fallo'])
   
   def execute(self, userdata):
      "execute method for Move state, moves forward until estado_debe_ser changes"
      global estado_debe_ser, pub
      rospy.loginfo("MOVE state (0): Green Light - Moving Forward")
      
      rate = rospy.Rate(10)
      move = Twist()
      
      # Move forward while estado_debe_ser is 0
      # 0 means Green light
      # 1 means Red light
      while not rospy.is_shutdown() and estado_debe_ser == 0:
            move.linear.x = 0.2
            move.angular.z = 0.0
            pub.publish(move)
            rate.sleep()
      
      # If we exit the while loop, it means estado_debe_ser changed to 1 (or ROS was shut down)
      rospy.loginfo("MOVE state (1): Red Light - Stopping.")
      move.linear.x = 0.0
      pub.publish(move) 
      return 'exito'


class EvitaObstaculo(smach.State):
    "evitar obstaculo state: performs a blind turn to avoid obstacle"
    def __init__(self):
        smach.State.__init__(self, outcomes=['exito','fallo'])
      
    def execute(self, userdata):
        "execute method for EvitaObstaculo state, performs a blind turn"
        global pub, estado_debe_ser
        rospy.loginfo("Executing EVITA state: Performing blind turn to avoid obstacle.")
        
        move = Twist()
        move.linear.x = 0.0
        move.angular.z = 0.5 
        
        # Publish the command for a fixed duration
        tiempo_inicio = rospy.get_time()
        while (rospy.get_time() - tiempo_inicio) < 2.0:
            pub.publish(move)
            rospy.sleep(0.1)
            
        # Stop the robot after turning
        move.angular.z = 0.0
        pub.publish(move)
        rospy.loginfo("EVITA state: Maneuver completed.")
        rospy.loginfo("RESETTING ALARM: Back to Green Light (0)")
        estado_debe_ser = 0
        
        return 'exito' 

def monitor(msg):
   "Monitor callback to check front distances"
   global estado_debe_ser
   distancia_frente = msg.ranges[0]
   if distancia_frente == float('inf'):
      distancia_frente = 10.0
   rospy.logwarn("MONITOR: (%.2fm)", distancia_frente)
   # logic to change state
   if distancia_frente < 0.6:
         if estado_debe_ser == 0:
            rospy.logwarn("MONITOR: Detect (%.2fm) -> CAMBIANDO ESTADO", distancia_frente)
            estado_debe_ser = 1


      
#Modulo principal:
rospy.init_node('arquitectura_navegacion')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.Subscriber('/scan', LaserScan, monitor)
estado_debe_ser=0


#creamos la maquina de estados compuesta
maquina_compuesta=smach.StateMachine(outcomes=['exito','fallo'])
muevete_state=Muevete()
evita_obstaculo_state=EvitaObstaculo()


with maquina_compuesta:
   #agregamos estados a la maquina de estados compuesta
   smach.StateMachine.add('MUEVETE',muevete_state,transitions={'exito':'EVITAOBSTACULO','fallo':'fallo'})
   smach.StateMachine.add('EVITAOBSTACULO',evita_obstaculo_state,transitions={'exito':'MUEVETE','fallo':'fallo'})
   
#ejecutamos la maquina de estados compuesta:
maquina_compuesta.execute()

