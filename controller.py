#!/user/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np



rospy.Subscriber("wl", Float32, wl_cb)  

rospy.Subscriber("wr", Float32, wr_cb) 

x,y,theta = rospy.Publisher('cmd_vel', Float32, queue_size=1)