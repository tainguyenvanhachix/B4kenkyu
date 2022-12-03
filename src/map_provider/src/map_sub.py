#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from turtlebot_tasp.msg import mapdataArray, mapdataPoint

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data[0:10])
    # print(data.info.width)
    # print(len(data.data))
    pass

def callback2(data):
    print('wall')
    print(data)

def callback3(data):
    print('BTP')
    print(data[-1])

def listener():

    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber('map', OccupancyGrid, callback)
    rospy.Subscriber('wall', mapdataArray, callback2)
    rospy.Subscriber('BTP', mapdataArray, callback3)

    rospy.spin()

if __name__ == '__main__':
    listener()
