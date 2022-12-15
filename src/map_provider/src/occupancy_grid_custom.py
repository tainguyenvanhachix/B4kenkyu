#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

limitations = [(-1,-2),(-1,-1),(-1,0),(-1,1),(-1,2),(-1,3),(-1,4),(0,-2),(1,-2),(2,-2),(3,-2),(4,-2),(5,-2),(5,-1),(5,0),(5,1),(5,2),(5,3),(5,4),(4,4),(3,4),
(2,4),(1,4),(0,4)]
walls = [(3,0),(3,-1),(1,1),(1,2)]
BTPs = [(-1,-2),(-1,-1),(-1,0),(-1,1),(-1,2),(-1,3),(-1,4),(0,-2),(1,-2),(2,-2),(3,-2),(4,-2),(5,-2),(5,-1),(5,0),(5,1),(5,2),(5,3),(5,4),(4,4),(3,4),
(2,4),(1,4),(0,4),(0,0),(0,1),(0,2),(0,-1),(1,-1),(1,0),(2,-1),(2,0),(3,-1),(3,0),(2,2),(1,2),(-2,0),(-2,1),(-2,2),(-2,3),(-2,4)]


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
        mapmsg.data.append(0)

    rate = rospy.Rate(10)
    rate.sleep()
    pub.publish(mapmsg)

def publisher_limitation():
    global mapmsg

    for limitation in limitations:
        (x,y) = limitation
        mapmsg.data[position_in_matrix(x,y)] = 60

    rate = rospy.Rate(10)
    rate.sleep()
    pub.publish(mapmsg)

def publisher_wall():
    global mapmsg

    for wall in walls:
        (x,y) = wall
        mapmsg.data[position_in_matrix(x,y)] = 100

    rate = rospy.Rate(10)
    rate.sleep()
    pub.publish(mapmsg)

def publisher_BTP():
    global mapmsg

    for BTP in BTPs:
        (x,y) = BTP
        mapmsg.data[position_in_matrix(x,y)] = 0

    rate = rospy.Rate(10)
    rate.sleep()
    pub.publish(mapmsg)

def pub_maker():
    global sum_marker, marker1, marker2, line_point, second_line_point
    
    sum_marker = MarkerArray()
    marker1 = Marker()
    marker2 = Marker()
    sum_marker.markers = [marker1,marker2]

    # Maker1 init
    marker1.header.frame_id = "map"
    marker1.type = marker1.LINE_STRIP
    marker1.action = marker1.ADD
    marker1.id = 1
    # marker1 scale
    marker1.scale.x = 0.03
    marker1.scale.y = 0.0
    marker1.scale.z = 0.0
    # marker1 color
    marker1.color.a = 1.0
    marker1.color.r = 1.0
    marker1.color.g = 0.0
    marker1.color.b = 0.0
    # marker1 orientaiton
    marker1.pose.orientation.x = 0.0
    marker1.pose.orientation.y = 0.0
    marker1.pose.orientation.z = 0.0
    marker1.pose.orientation.w = 1.0
    # marker1 position
    marker1.pose.position.x = 0.0
    marker1.pose.position.y = 0.0
    marker1.pose.position.z = 0.0
    # marker1 line points
    marker1.points = []
    # first point
    line_point = Point()
    line_point.x = 0.0
    line_point.y = 0.0
    line_point.z = 0.0
    marker1.points.append(line_point)
    # second point
    second_line_point = Point()
    second_line_point.x = 1.0
    second_line_point.y = 0.0
    second_line_point.z = 0.0
    marker1.points.append(second_line_point)

    # Maker2 init
    marker2.header.frame_id = "map"
    marker2.type = marker2.SPHERE_LIST
    marker2.action = marker2.ADD
    marker2.id = 2
    # marker2 scale
    marker2.scale.x = 0.05
    marker2.scale.y = 0.05
    marker2.scale.z = 0.05
    # marker2 color
    marker2.color.a = 1.0
    marker2.color.r = 1.0
    marker2.color.g = 1.0
    marker2.color.b = 0.0
    # marker2 orientaiton
    marker2.pose.orientation.x = 0.0
    marker2.pose.orientation.y = 0.0
    marker2.pose.orientation.z = 0.0
    marker2.pose.orientation.w = 1.0
    # marker2 position
    marker2.pose.position.x = 0.0
    marker2.pose.position.y = 0.0
    marker2.pose.position.z = 0.0
    # marker2 line points
    marker2.points = []
    # start point
    sphere_point = Point()
    sphere_point.x = 0.0
    sphere_point.y = 0.0
    sphere_point.z = 0.0
    marker2.points.append(sphere_point)

    rate = rospy.Rate(5)
    rate.sleep()
    trajectory_line_maker.publish(sum_marker)

def position_in_matrix(x,y):
    return int((y+49)*100 + (49+x))

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('map', OccupancyGrid, queue_size=1000)
        trajectory_line_maker = rospy.Publisher('trajectory_marker', MarkerArray, queue_size=1000)
        rospy.init_node('occupancy_grid', anonymous=True)
        publisher()
        publisher_limitation()
        publisher_wall()
        # publisher_BTP()
        # pub_maker()
    except rospy.ROSInterruptException:
        pass
