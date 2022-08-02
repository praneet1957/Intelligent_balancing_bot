#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np







def PID(phi,phi_dot):
    inp = np.array([[phi],[phi_dot]])
    l0 = np.array([[ 0.42332685 ,-0.20222105],
 [-0.68782717,  0.15276884],
 [-0.73504007  ,0.04829184],
 [-0.6999695 , -0.38863742],
 [-0.15463851,  0.11431317],
 [ 0.4975862  , 0.49425155]]
)

    l1 = np.array([[ 0.24514021 ,-0.63441986, -0.8702627  , 0.653707  ,  0.77872163 , 0.88055533],
 [-0.3653803 ,  0.84004796,  0.79240024,  0.42975596 , 0.22249372 ,-0.18821959],
 [-0.23613094 ,-0.166874 ,  -0.22761439, -0.13526174, -0.60490125 , 0.17999525]])

    l2 = np.array([[ 0.96543264 ,-0.6907096  ,-0.34969094]]


)

    b0 = np.array([ 0.12446024 ,-0.12310111 ,-0.12404072,  0.1154175,   0.12996078  ,0.12572351]


)
    b1 = np.array([ 0.11932582, -0.12067153 ,-0.12048591]
)
    b2 = np.array([0.11629402]



)

    s0 = np.dot(l0,inp)

    for i in range(len(b0)):
        s0[i] = s0[i] + b0[i]

    s1 = np.dot(l1,s0)

    for i in range(len(b1)):
        s1[i] = s1[i] + b1[i]

    s2 = np.dot(l2,s1)

    for i in range(len(b2)):
        s2[i] = s2[i] + b2[i]




    return s2[0]*2


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
    print(pid_value)
    

    


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