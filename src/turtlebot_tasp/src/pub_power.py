#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

def pub_power():
    pub= rospy.Publisher('motor_power', Bool, queue_size=10)
    rospy.init_node('power_node', anonymous=False)
    bool=Bool()
    bool.data=1
    pub.publish(bool)

if __name__ == '__main__':
    try:
        pub_power()
    except rospy.ROSInterruptException:
        pass
