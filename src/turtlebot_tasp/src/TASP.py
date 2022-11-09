#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.1
ANGULAR_VEL= -0.785
TURN_DISTANCE = 0.3
LIDAR_ERROR = 0.05
SAFE_TURN_DISTANCE = TURN_DISTANCE + LIDAR_ERROR

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
        samples = len(scan.ranges)
        samples_view = 1

        if samples_view > samples:
            samples_view = samples

        if samples_view == 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    def obstacle(self):
        twist = Twist()
        straight_moving = True

        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            if min_distance < SAFE_TURN_DISTANCE:
                if straight_moving:
                    twist.linear.x = 0.0
                    twist.angular.z = ANGULAR_VEL
                    self._cmd_pub.publish(twist)
                    rospy.loginfo('Turned')
                    rospy.sleep(2)
            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                rospy.loginfo('Distance of the obstacle : %f', min_distance)

def main():
    rospy.init_node('TASP')
    try:
        obstacle = Obstacle()
    except rospy.ROSInitException:
        pass

if __name__== '__main__':
    main()
