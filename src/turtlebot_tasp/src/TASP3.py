#!/usr/bin/env python3
import rospy
import math
from math import sqrt, pow, pi, atan2
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion

SAMPLES_NUMBER = 1
CELL_LENGTH = 0.310
SAFE_DISTANCE = 0.530
LINEAR_VEL = 0.05
ANGULAR_VEL = 0.2

class GoAndTurn():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel',Twist,queue_size=5)
        self.X = 0
        self.Y = 0
        self.goal_z = 0
        self.position = Point()
        self.move_cmd = Twist()
        self.r = rospy.Rate(10)
        self.last_rotation = 0
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(1.0))
        (self.position, self.rotation) = self.get_odom()
        self.check_around_and_go()

    def check_around_and_go(self):
        btp = 1
        print('check_around')
        # btp = self.back_tracking_point()
        while btp > 0:
            print('start_while_btp')
            front_min_distance = min(self.get_scan(0))
            left_min_distance = min(self.get_scan(90))
            right_min_distance = min(self.get_scan(-90))
            turtlebot_direction = self.check_direction()
            print('turtlebot_direction: '+turtlebot_direction)
            if front_min_distance > SAFE_DISTANCE:
                print('front_dis: '+str(front_min_distance))
                if turtlebot_direction == 'x_positive':
                    self.X +=1; right_point = self.Y - 1; left_point = self.Y + 1; center_point = self.X
                    print('self.X: '+str(self.X))
                elif turtlebot_direction == 'y_positive':
                    self.Y +=1; right_point = self.X + 1; left_point = self.X - 1; center_point = self.Y
                elif turtlebot_direction == 'x_negative':
                    self.X -=1; right_point = self.Y + 1; left_point = self.Y - 1; center_point = self.X
                elif turtlebot_direction == 'y_negative':
                    self.Y -=1; right_point = self.X - 1; left_point = self.X + 1; center_point = self.Y
                self.go_straight()
            elif left_min_distance > SAFE_DISTANCE and right_min_distance > SAFE_DISTANCE:
                print('right_dis: '+str(right_min_distance))
                right_distance = sqrt(pow((center_point), 2) + pow((right_point), 2))
                left_distance = sqrt(pow((center_point), 2) + pow((left_point), 2))
                if self.X == 0 or self.Y == 0:
                    if left_min_distance > right_min_distance:
                        self.turn('left')
                    else:
                        self.turn('right')
                elif left_distance > right_distance:
                    self.turn('left')
                else:
                    self.turn('right')
            elif left_min_distance > SAFE_DISTANCE:
                self.turn('left')
            elif right_min_distance > SAFE_DISTANCE:
                self.turn('right')
            else:
                self.point_to_point()
            print(' ')

    def go_straight(self):
        print('go_traight')
        last_distance = CELL_LENGTH
        goal_x = self.X*CELL_LENGTH
        goal_y = self.Y*CELL_LENGTH
        distance = sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))
        print('distance: '+str(distance))
        while distance > 0.005:
            (self.position, no_use) = self.get_odom()
            self.move_cmd.linear.x = LINEAR_VEL
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            distance = sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))
            if format(distance,".3f") > format(last_distance,".3f"):
                print('break')
                break
            last_distance = distance

    def turn(self,turn_direction):
        if turn_direction == 'left':
            self.goal_z += pi/2
        elif turn_direction == 'right':
            self.goal_z -= pi/2
        while abs(self.rotation - self.goal_z) > 0.01:
            (no_use, rotation) = self.get_odom()
            if (self.goal_z/(pi/2)) % 4 == 3 or (self.goal_z/(pi/2)) % 4 == -3:
                if (rotation - self.last_rotation) > pi:
                    round -=1
                elif (rotation - self.last_rotation) < -pi:
                    round +=1
            self.last_rotation = rotation
            self.rotation = rotation + round*2*pi
            if rotation <= self.goal_z:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = ANGULAR_VEL
            else:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = -ANGULAR_VEL
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()

    def point_to_point(self):
        print('COMMING SOON')
    def check_direction(self):
        if (self.goal_z/(pi/2)) % 4 == 0:
            direction = 'x_positive'
        elif (self.goal_z/(pi/2)) % 4 == 1 or (self.goal_z/(pi/2)) % 4 == -3:
            direction = 'y_positive'
        elif (self.goal_z/(pi/2)) % 4 == 2 or (self.goal_z/(pi/2)) % 4 == -2:
            direction = 'x_negative'
        elif (self.goal_z/(pi/2)) % 4 == 3 or (self.goal_z/(pi/2)) % 4 == -1:
            direction = 'y_negative'
        return direction

    def back_tracking_point():
        print('COMMING SOON')

    # Get scan_filter array that contains distance from lidar to obstacle
    def get_scan(self, scan_direction):
        scan = rospy.wait_for_message('scan', LaserScan)
        while scan.intensities[scan_direction]==0: # Reciving messages from scan topic if intensities are good
            scan = rospy.wait_for_message('scan', LaserScan)

        scan_filter = []
        samples = len(scan.ranges)
        samples_view = SAMPLES_NUMBER # Define how many sample want to get, 1 sample = 1 degree
        if samples_view > samples:
            samples_view = samples
        if samples_view == 1:
            scan_filter.append(scan.ranges[scan_direction])
        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2) + scan_direction
            right_lidar_samples_ranges = samples_view//2 + scan_direction
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)
        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        return scan_filter

    # Get self.position and rotation of turtlebot base on different bettween odom and base_footprint frame
    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
            rotation = euler_from_quaternion(rot)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), rotation[2])

if __name__ == '__main__':
    rospy.init_node('TASP3', anonymous=False)
    try:
        print('Start')
        GoAndTurn()
    except: rospy.loginfo("Shutdown program.")