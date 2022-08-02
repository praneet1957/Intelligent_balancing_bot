#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np







def PID(phi,phi_dot):
    inp = np.array([[phi],[phi_dot]])
    l0 = np.array([[ 1.1078708 , -0.74231887],
 [ 0.99579096,  0.27590486],
 [-0.50231266 , 0.5512891 ],
 [ 0.69410807 , 0.45252278],
 [-0.14690547 , 0.76593286],
 [-1.0632123  ,-0.05590719]]
)

    l1 = np.array([[-0.13899525 , 0.40430298 ,-0.39931855 ,-0.33509114 ,-0.22011217 , 1.0320369 ],
 [ 0.1187456   ,0.21746099 ,-0.2591049  , 0.19763418 , 0.01381929 ,-0.32188755],
 [ 0.79851663 , 0.5485398  , 0.03827827 , 0.01682098 , 0.15429612 ,-0.55945396]])

    l2 = np.array([[-0.40437916, -0.28670052 , 1.1316354 ]]

)

    b0 = np.array([ 0.10536943 , 0.07703248 , 0.08003894, -0.09178305 , 0.08056954, -0.10315816]

)
    b1 = np.array([-0.09136853 ,-0.09329836  ,0.10083265])
    b2 = np.array([0.10086089]

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