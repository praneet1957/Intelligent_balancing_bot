#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('/self_balancing_bot/jointL_velocity_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/self_balancing_bot/jointR_velocity_controller/command', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg1 = Float64()
    msg1.data = 0.5
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(msg1)
        pub2.publish(msg1)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
