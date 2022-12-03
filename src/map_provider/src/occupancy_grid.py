#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from turtlebot_tasp.msg import mapdataPoint

# limitations =[(-1,0),(-1,1),(-1,2),(-1,-1),(0,-1),(1,-1),(2,-1),(4,1),(4,3),(-1,3),(2,0),(2,1),(3,1),(4,2),(3,3),(2,3),(1,3),(0,3)] #1
# limitations =[(-1,-1),(1,-3),(3,-3),(3,2),(-1,2),(-1,0),(-1,1),(0,-1),(1,-1),(1,-2),(2,-3),(3,-2),(3,-1),(3,0),(3,1),(2,2),(1,2),(0,2)] #2
limitations =[(-1,-1),(4,-1),(4,4),(0,4),(-1,3),(-1,0),(-1,1),(-1,2),(0,-1),(1,-1),(2,-1),(3,-1),(4,0),(4,1),(4,2),(4,3),(3,4),(2,4),(1,4),(0,3)] #3

def publisher():
    global mapmsg

    mapmsg = OccupancyGrid()
    mapmsg.info.width = 100
    mapmsg.info.height = 100
    mapmsg.info.resolution = 0.31
    mapmsg.info.origin.position.x = -50*0.31+0.155
    mapmsg.info.origin.position.y = -50*0.31+0.155
    mapmsg.info.origin.position.z = 0.0
    mapmsg.info.origin.orientation.x = 0.0
    mapmsg.info.origin.orientation.y = 0.0
    mapmsg.info.origin.orientation.z = 0.0
    mapmsg.info.origin.orientation.w = 1.0

    for i in range(10000):
        mapmsg.data.append(-1)

    rate = rospy.Rate(4)
    rate.sleep()
    pub.publish(mapmsg)

def publisher_limitation():
    global mapmsg

    for limitation in limitations:
        (x,y) = limitation
        mapmsg.data[position_in_matrix(x,y)] = 70

    rate = rospy.Rate(4)
    rate.sleep()
    pub.publish(mapmsg)

def publisher_wall(data):
    global mapmsg

    mapmsg.data[position_in_matrix(data.x,data.y)] = 100

    rate = rospy.Rate(4)
    rate.sleep()
    pub.publish(mapmsg)

def publisher_BTP(data):
    global mapmsg

    mapmsg.data[position_in_matrix(data.x,data.y)] = 0

    rate = rospy.Rate(4)
    rate.sleep()
    pub.publish(mapmsg)

def position_in_matrix(x,y):
    return int((y+49)*100 + (49+x))

# def callback_map(data):
#     pass

def subscriber():
    # rospy.Subscriber('map', OccupancyGrid, callback_map)
    rospy.Subscriber('wall', mapdataPoint, publisher_wall)
    rospy.Subscriber('BTP', mapdataPoint, publisher_BTP)

    rospy.spin()

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('map', OccupancyGrid, queue_size=1000)
        rospy.init_node('occupancy_grid', anonymous=True)
        publisher()
        publisher_limitation()
        subscriber()
    except rospy.ROSInterruptException:
        pass
