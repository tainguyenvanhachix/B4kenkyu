#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def pub_vel():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('vel_node', anonymous=False)
    twist=Twist()
    twist.linear.x=0.00
    twist.angular.z=0.0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        pub_vel()
    except rospy.ROSInterruptException:
        pass
