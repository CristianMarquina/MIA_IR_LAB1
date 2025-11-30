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
   def __init__(self):
      smach.State.__init__(self,outcomes=['exito','fallo'])
      
   def execute (self,userdata):
      print ("paso por el metodo execute de muevete ")
      a=True
      if  (a):
         rate=rospy.Rate(4)
         while (not rospy.is_shutdown() and estado_debe_ser==0):
            rate.sleep()
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
      

#Creamos aqui la funcion monitor
def monitor(msg):
   global estado_debe_ser
   print ("he recibido mensaje")
   estado_debe_ser=1

      
#Modulo principal:
rospy.init_node('arquitectura_navegacion')
estado_debe_ser=0


#creamos la maquina de estados compuesta
maquina_compuesta=smach.StateMachine(outcomes=['exito','fallo'])
muevete_state=Muevete()
evita_obstaculo_state=EvitaObstaculo()

#iniciamos el hilo del monitoreo del laser, en este caso representado por un topic basico
rospy.Subscriber('/aviso',Int16,monitor)

with maquina_compuesta:
   #agregamos estados a la maquina de estados compuesta
   smach.StateMachine.add('MUEVETE',muevete_state,transitions={'exito':'EVITAOBSTACULO','fallo':'fallo'})
   smach.StateMachine.add('EVITAOBSTACULO',evita_obstaculo_state,transitions={'exito':'exito','fallo':'fallo'})
   
#ejecutamos la maquina de estados compuesta:
maquina_compuesta.execute()

