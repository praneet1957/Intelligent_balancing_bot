#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np


global desired 
global prev_pid
global err_int
global start
global stop
global cont
global file

start = 0
stop = 0
cont = 0

err_int = 0 
desired = 0
prev_pid = 0 

file = open("My_file5.txt","a")


def PID(phi,phi_dot):
    global prev_pid
    global desired
    global err_int

    delta_t=0.1
    max_vel = 2
    Kp = 10
    Kd = 1
    Ki = 0.05

    err = desired - phi
    err_int = err_int + err*delta_t

    pwm = Kp*err - Kd*phi_dot + Ki*err_int
    pwm = pwm + prev_pid

    if pwm > max_vel:
        pwm = max_vel

    elif pwm < -max_vel:
        pwm = -max_vel

    else:
        pwm = pwm

    return pwm


def callback(data):
    global prev_pid
    global start
    global stop
    global cont
    global file

    q_x = data.orientation.x
    q_y = data.orientation.y
    q_z = data.orientation.z
    q_w = data.orientation.w
    phi_dot = data.angular_velocity.x


    phi = np.arctan((2*(q_w*q_x+q_y*q_z))/(1-2*(q_y**2 + q_x**2)))
    pid_value = PID(phi,phi_dot)

    pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)


    rospy.init_node('pid_control', anonymous=True)
    rate = rospy.Rate(50) 
    
    

    #while not rospy.is_shutdown():
    #rospy.loginfo(pid_value)
    rospy.loginfo(phi)
    
    

    

    # if (abs(phi)< 3*3.14/180) and (cont == 0):
    #     start = rospy.get_time()
    #     cont =1
        

    # elif (abs(phi)< 3*3.14/180) and (cont == 1):
    #     cont = 1
    #     stop = rospy.get_time()
        
    #     if (stop-start)>2:
    #         pid_value = pid_value/2
    #         print(pid_value)

    # else:
    #     cont=0
    #     start = 0

    


    vel_msg=Twist()
    vel_msg.linear.x = -pid_value
    vel_msg.angular.z = 0
    pub.publish(vel_msg)



    prev_pid = pid_value
    L = str(-pid_value) + " " + str(phi) +" "+ str(phi_dot) +"\n"
    file.write(L)
    rate.sleep()
     



def control():
	rospy.init_node('pid_control' , anonymous=True)
	rospy.Subscriber("imu" , Imu , callback)
	rospy.spin()
    
	



if __name__ == '__main__':
    control()