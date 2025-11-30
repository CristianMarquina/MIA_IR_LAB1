#!/usr/bin/env python
#Este es un ejemplo basico, con dos estados


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys
import math
import numpy as np
import smach


def scan_callback(msg, userdata):
    userdata.laser = msg
    userdata.laser_available = True


#Hay una parte inicial del codigo para comprobar sentido de las medidas del laser
#se dividen las lecturas en 8 sectores
#se calcula la fuerza repulsiva total (modulo y signo) como resultado de calcular las fuerzas repulsivase en el primer y el ultimo sector (parte frontal del robot)

      
class Muevete(smach.State):
   def __init__(self):
      smach.State.__init__(self,outcomes=['exito','fallo'])
      
   def execute (self,userdata):
      print ("paso por el metodo execute de muevete ")
      a=True
      if  (a):
         return ('exito')
      else:
         return ('fallo')
      
class EvitaObstaculo(smach.State):
   def __init__(self):
      smach.State.__init__(self,outcomes=['exito','fallo'])
      
      
   def execute(self,userdata):
      print ("paso por el metodo execute de evita obstaculo ")
      b=True
      if (b):
         return ('exito')
      else:
         
         return ('fallo')
      

      
#Modulo principal:
rospy.init_node('arquitectura_navegacion')

#creamos la maquina de estados compuesta
maquina_compuesta=smach.StateMachine(outcomes=['exito','fallo'])
muevete_state=Muevete()
evita_obstaculo_state=EvitaObstaculo()

with maquina_compuesta:
   #agregamos estados a la maquina de estados compuesta
   smach.StateMachine.add('MUEVETE',muevete_state,transitions={'exito':'EVITAOBSTACULO','fallo':'fallo'})
   smach.StateMachine.add('EVITAOBSTACULO',evita_obstaculo_state,transitions={'exito':'exito','fallo':'fallo'})
   
#ejecutamos la maquina de estados compuesta:
maquina_compuesta.execute()

