#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np







def PID(phi,phi_dot):
    inp = np.array([[phi],[phi_dot]])
    l0 = np.array([[ 0.4554882  , 0.2862286 ],
 [ 0.49811277 ,-0.15839161],
 [-0.6031759 ,  0.56562775],
 [ 0.21957557 ,-0.8133691 ],
 [-0.17290847 , 0.16817544],
 [ 0.49916074 , 0.30181316]]
)

    l1 = np.array([[-0.34799507, -0.8510442,  -0.27919084, -0.35122964 ,-0.3889998 , -0.77011436],
 [-0.02780313,  0.32000273, -0.30261597, -0.2457823,   0.53145707 , 0.33922425],
 [ 0.99378073 , 0.13195366, -0.9569037 ,  0.719649 ,  -0.33851513 , 0.45301524]])

    l2 = np.array([[-1.3955364,   0.43512788,  0.52907926]]
)

    b0 = np.array([0.06666108 ,0.06686694 ,0.06869654 ,0.06914597, 0.06707546 ,0.06677959]
)
    b1 = np.array([-0.06725737  ,0.06504581  ,0.06773554])
    b2 = np.array([0.06751213]
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