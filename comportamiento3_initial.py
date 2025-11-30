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

      
class Muevete(smach.State):
   def __init__(self):
      smach.State.__init__(self,outcomes=['exito','fallo'],input_keys=['valor1','valor2'],output_keys=['valor_final'])
      
   def execute (self,userdata):
      print ("paso por el metodo execute de muevete ")
      a=True      
      if  (a):
         print ("he recibido los datos de entrada ",userdata.valor1,userdata.valor2)          
         userdata.valor_final=(int)(userdata.valor1)+(int)(userdata.valor2)
         rate=rospy.Rate(4)
         while (not rospy.is_shutdown() and (int)(userdata.valor1)<3999):
            rate.sleep()
         print ("valor revisado antes de salir ",userdata.valor1)
         return ('exito')
      else:
         return ('fallo')
      
class EvitaObstaculo(smach.State):
   def __init__(self):
      smach.State.__init__(self,outcomes=['exito','fallo'],input_keys=['valor_final'])
      
      
   def execute(self,userdata):
      print ("paso por el metodo execute de evita obstaculo ")
      print ("valor que ha entrado es",userdata.valor_final)
      b=True
      if (b):
         return ('exito')
      else:
         return ('fallo')
      

#Creamos aqui la funcion monitor
def monitor(msg,userdata):
   #global estado_debe_ser
   print ("he recibido mensaje")
   estado_debe_ser=1
   print ("cambiamos userdata")
   userdata.valor1=4000

      
#Modulo principal:
rospy.init_node('arquitectura_navegacion')
estado_debe_ser=0


#creamos la maquina de estados compuesta
maquina_compuesta=smach.StateMachine(outcomes=['exito','fallo'])
muevete_state=Muevete()
evita_obstaculo_state=EvitaObstaculo()

numero1=input('introduce un dato para pasarle a la maquina de estados ')
numero2=input('introduce un segundo dato para pasarle a la maquina de estados ')

maquina_compuesta.userdata.valor1=numero1
maquina_compuesta.userdata.valor2=numero2

#iniciamos el hilo del monitoreo del laser, en este caso representado por un topic basico
rospy.Subscriber('/aviso',Int16,monitor,callback_args=(maquina_compuesta.userdata))

with maquina_compuesta:
   #agregamos estados a la maquina de estados compuesta
   smach.StateMachine.add('MUEVETE',muevete_state,transitions={'exito':'EVITAOBSTACULO','fallo':'fallo'})
   smach.StateMachine.add('EVITAOBSTACULO',evita_obstaculo_state,transitions={'exito':'exito','fallo':'fallo'})
   
#ejecutamos la maquina de estados compuesta:
maquina_compuesta.execute()

