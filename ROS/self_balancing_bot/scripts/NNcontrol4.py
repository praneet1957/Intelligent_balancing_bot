#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np







def PID(phi,phi_dot):
    inp = np.array([[phi],[phi_dot]])
    l0 = np.array([[ 0.34350762  ,0.6904315 ],
 [-0.9302461 , -0.30880463],
 [ 0.9328244 , -0.13712992],
 [-0.68449986, -0.15929642],
 [-0.48899072, -0.43943554],
 [ 0.0747711 ,  0.13345064]]
)

    l1 = np.array([[ 0.7216828,   0.7172091,  -0.12915178,  0.5020005 ,  0.71049994, -0.61085296],
 [ 0.2947037   ,0.17299977 ,-0.34943974 , 0.28850633 , 0.5414762 , -0.37333733],
 [ 0.35974437,  0.6580474 , -0.7368522 ,  0.34410933 ,-0.52196103 , 0.2872262 ]]
)

    l2 = np.array([[-0.8754946 ,-0.578745 , -0.7894478]]

)

    b0 = np.array([-0.09139024 ,-0.09502025 , 0.09500314, -0.09603326, -0.09953524,  0.09710996]

)
    b1 = np.array([-0.09352331 ,-0.09394647 ,-0.09263705]

)
    b2 = np.array([0.09314661]


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