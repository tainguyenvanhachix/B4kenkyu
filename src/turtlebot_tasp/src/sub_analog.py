#!/usr/bin/env python3
import rospy
from turtlebot3_msgs.msg import SensorState

current = 0
count = 0

def callback(data):
    global count, current
    if count < 5:
        current += data.sonar
        count +=1
    else:
        count = 0
        current = current/5
        current = (current)
        rospy.loginfo(current)
        current = 0

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('sensor_state', SensorState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
