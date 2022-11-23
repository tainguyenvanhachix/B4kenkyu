#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    print('90 degree:'+str(data.ranges[0])+'m ')
    print('intensities: '+str(data.intensities[0]))

def sub_scan():

    rospy.init_node('scan_node', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    sub_scan()
