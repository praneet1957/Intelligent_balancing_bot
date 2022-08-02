#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np







def PID(phi,phi_dot):
    inp = np.array([[phi],[phi_dot]])
    l0 = np.array([[-0.7410362 , -0.23503788],
          [-0.4714638 , -0.42556232],
          [-0.21118632, -0.7174453 ],
          [-0.67961365 , 0.48678756],
          [-0.38048935 , 0.490922  ],
          [-0.85989195  ,0.44628268]])

    l1 = np.array([[-0.25266886 , 0.9715847  , 0.87220806 , 0.61759263 , 0.0762684  , 0.32495278],
          [ 0.4258451 , -0.30404302 ,-0.8973192 , -0.8274297 , -0.13418795 ,-0.32117936],
          [-1.0514956 , -0.86062634 , 0.56169903, -0.34807372, -0.7358583,  -0.25191778]])

    l2 = np.array([[-0.27449745 , 0.64643526  ,0.67852676]])

    b0 = np.array([ 0.18001944 ,-0.15742329, -0.1712196,  -0.13717619, -0.13243796, -0.11890967])
    b1 = np.array([-0.19090496 , 0.14200234,  0.13696307])
    b2 = np.array([0.14164177])

    s0 = np.dot(l0,inp)

    for i in range(len(b0)):
        s0[i] = s0[i] + b0[i]

    s1 = np.dot(l1,s0)

    for i in range(len(b1)):
        s1[i] = s1[i] + b1[i]

    s2 = np.dot(l2,s1)

    for i in range(len(b2)):
        s2[i] = s2[i] + b2[i]




    return s2[0]


def callback(data):
    

    q_x = data.orientation.x
    q_y = data.orientation.y
    q_z = data.orientation.z
    q_w = data.orientation.w
    phi_dot = data.angular_velocity.x


    phi = np.arctan((2*(q_w*q_x+q_y*q_z))/(1-2*(q_y**2 + q_x**2)))
    pid_value = PID(phi,phi_dot)

    pub= rospy.Publisher('/cmd_vel', Twist,queue_size=10)


    rospy.init_node('pid_control', anonymous=True)
    rate = rospy.Rate(50) 
    
    

    #rospy.loginfo(pid_value)
    rospy.loginfo(phi)
    
    

    


    vel_msg=Twist()
    vel_msg.linear.x = pid_value
    vel_msg.angular.z = 0
    pub.publish(vel_msg)
    rate.sleep()
     



def control():
	rospy.init_node('pid_control' , anonymous=True)
	rospy.Subscriber("imu" , Imu , callback)
	rospy.spin()
    
	



if __name__ == '__main__':
    control()