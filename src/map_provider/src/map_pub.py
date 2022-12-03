#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

def talker():
    pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    mapmsg = OccupancyGrid()
    mapmsg.info.width = 310
    mapmsg.info.height = 310
    mapmsg.info.resolution = 0.01
    mapmsg.data = []
    for i in range(96100):
        mapmsg.data.append(50)
    for i in range(50000,55000):
        mapmsg.data[i] = -1

    
    rate = rospy.Rate(5)
    rate.sleep()
    pub.publish(mapmsg)
    # while not rospy.is_shutdown():
    # #     hello_str = "hello world %s" % rospy.get_time()
    # #     rospy.loginfo(hello_str)
    #     pub.publish(mapmsg)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
