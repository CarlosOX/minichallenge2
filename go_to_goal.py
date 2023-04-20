#!/usr/bin/env python  

import rospy  

from geometry_msgs.msg import Twist 

from std_msgs.msg import Float32 

import numpy as np 

 

#This class will make the puzzlebot move following a square 

class GoToGoal():  

    def __init__(self):  

        rospy.on_shutdown(self.cleanup) 

        ############ ROBOT CONSTANTS ################  

        r=0.05 #wheel radius [m] 

        L=0.19 #wheel separation [m] 

        theta = 0  #robot orientation in radians

        x_k = 0  #robot position along the x-axis
        
        y_k = 0 # robot position along the y-axis

        v = 0.0 # robot linear speed (m/s)

        w = 0.0 #robot angular speed (rad/s)

        self.wl = 0
        self.wr = 0



        while rospy.get_time() == 0:
            print("no simulated time has been received yet")
        print("Got time")


        previous_time = rospy.get_time()

        ###########  INIT PUBLISHERS ################ 

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        ############## SUBSCRIBERS ##################  

        rospy.Subscriber("wl", Float32, self.wl_cb)  

        rospy.Subscriber("wr", Float32, self.wr_cb)  

        x_G = 1.0
        y_G=  -1.0

        v_msg=Twist() 

        rate = rospy.Rate(20) #20Hz  

        

        print("Node initialized") 

        while not rospy.is_shutdown():  

            delta_t = rospy.get_time() - previous_time

            previous_time = rospy.get_time()

            v = r * (self.wr + self.wl) / 2.0

            w = r * (self.wr - self.wl) / L

            theta = theta + w * delta_t
            
            theta = np.arctan2(np.sin(theta),np.cos(theta))
            

            x_k = x_k + v * np.cos(theta) * delta_t

            y_k = y_k + v *np.sin(theta) * delta_t

            #distance to goal

            d = np.sqrt((x_G-x_k)**2 + (y_G-y_k)**2)

            #ANgle to the goal
            theta_g = np.arctan2(y_G - y_k , x_G - x_k)

            etheta = theta_g - theta


             
            print("x_k : " + str(x_k))
            print("y_k : " + str(x_k))
            print("theta : " + str(theta))

            print("e_theta"+ str(etheta))
            print("d: "+ str(d))


            #pub_cmd_vel.publish(v_msg) #publish the robot's speed  

            rate.sleep()  

           
    def wl_cb(self, wl):  

        ## This function receives the left wheel speed from the encoders  

        self.wl = wl.data 

         

    def wr_cb(self, wr):  

        ## This function receives the right wheel speed from the encoders 

        self.wr = wr.data  

         

    def cleanup(self):  

        #This function is called just before finishing the node  

        # Y ou can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.    

        vel_msg = Twist() 

        self.pub_cmd_vel.publish(vel_msg) 

 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("go_to_goal", anonymous=True)  

    GoToGoal()  