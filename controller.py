#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 


#Setup parameters, variables and callback functions here (if required)
control_signal = 0.0

prev_error = 0.0
integral_term = 0.0
posicion_o = 0  #Posicion de salida de la planta
reference = 0


e=[0.0,0.0,0.0]
#e[0] error actual    e[1] error anterior    e[2] error dos veces anterior
u=[0.0,0.0]
#u[0] salida actual    u[1] salida anterior
Ts = 0.4
#periodo de muestreo

#ganancias del modelo discreto
kp = rospy.get_param("/proportional",0.0)
ki = rospy.get_param("/integral",0.0)
kd = rospy.get_param("/derivative",0.0)

l = 1

K1=kp + Ts*ki + kd/Ts
K2=-kp - 2.0*kd/Ts
K3=kd/Ts

# definir la funcion de callback 


def callback_reference(msg):
  global reference, reference_t
  reference = msg.reference
  reference_t = msg.time

def callback_motor_output(msg):
  global motor_o
  motor_o = msg.data 
  



#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(150)
    rospy.on_shutdown(stop)
    rospy.Subscriber("/set_point", set_point, callback_reference)
    rospy.Subscriber("/motor_output", Float32, callback_motor_output)
    motor_input_pub = rospy.Publisher("/motor_input", Float32, queue_size=10)

    #motor_o = motor_o / 13.08996939
    #reference = reference / 13.08996939
    
    #Definir el error inicial
    prev_error = 0
    integral_term = 0



    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():
        #Write your code here
        
        e[0]=reference - motor_o
        u[0]=K1*e[0]+K2*e[1]+K3*e[2]+u[1]
        

        if(u[0]>l):
          u[0]=l
        if(u[0]<-l):
          u[0]=-l

        e[2]=e[1]
        e[1]=e[0]
        u[1]=u[0]

        
        #rospy.loginfo(control_signal)
        
        
        motor_i = u[0]
        #motor_i.time = rospy.get_time()
        motor_input_pub.publish(motor_i)
       


