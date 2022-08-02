#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def stop(vel_msg):
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

def move():
    pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
    rospy.init_node('move', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    vel_msg=Twist()
    while not rospy.is_shutdown():
        vel_msg.linear.x = -1.0
        vel_msg.angular.z = 2.0
        print('upar')
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass