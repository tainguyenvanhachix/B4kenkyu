#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

def talker():
    pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    mapmsg = OccupancyGrid()
    mapmsg.data = [0,0,100]

    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
        pub.publish(mapmsg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
