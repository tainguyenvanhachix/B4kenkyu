#!/usr/bin/env python3
import math
from math import atan2, pi, pow, sqrt

import rospy
import tf
from geometry_msgs.msg import Point, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

SAMPLES_NUMBER = 1
CELL_LENGTH = 0.310
SAFE_DISTANCE = 0.550
SAFE_DISTANCE_SIDE = SAFE_DISTANCE - 0.064
SAFE_DISTANCE_BACK = SAFE_DISTANCE_SIDE - 2*0.064
LINEAR_VEL = 0.05
ANGULAR_VEL = 0.2
QUAT_90DEGREE = quaternion_from_euler (0, 0, pi/2)
OMEGA_0 = 0.05
KA1 = 0.5
KA2 = 0.5

class GoAndTurn():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel',Twist,queue_size=5)
        self.X = 0
        self.Y = 0
        self.goal_quat_z = 0
        self.goal_quat_w = 1
        self.trajectory = []
        self.wall =[]
        self.back_tracking_point = [(0,0)]
        self.position = Point()
        self.quat = Quaternion()
        self.move_cmd = Twist()
        self.r = rospy.Rate(20)
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(1.0))
        (self.position, self.quat) = self.get_odom()
        self.check_around_and_go()

    def check_around_and_go(self):
        if min(self.get_scan(-180)) < SAFE_DISTANCE_BACK:
            self.wall.append((self.X-1,self.Y))
        front_min_distance = min(self.get_scan(0))
        left_min_distance = min(self.get_scan(90))
        right_min_distance = min(self.get_scan(-90))
        turtlebot_direction = self.check_direction()
        print('turtlebot_direction: '+ turtlebot_direction)
        self.add_wall(turtlebot_direction, front_min_distance, left_min_distance, right_min_distance)
        self.add_trajectory()
        self.add_back_tracking()
        print('self.wall: ')
        print(self.wall)
        print('self.trajectory: ')
        print(self.trajectory)
        print('self.back_tracking_point: ')
        print(self.back_tracking_point)
        while len(self.back_tracking_point) > 0:
            (front_X,front_Y) = self.check_front(self.X,self.Y,turtlebot_direction)
            (left_X,left_Y) = self.check_left(self.X,self.Y,turtlebot_direction)
            (right_X,right_Y) = self.check_right(self.X,self.Y,turtlebot_direction)
            if (front_X,front_Y) not in self.back_tracking_point:
                self.move_cmd.linear.x = 0
                self.cmd_vel.publish(self.move_cmd)
            if (front_X,front_Y) in self.back_tracking_point:
                print('front_dis: '+str(front_min_distance))
                if turtlebot_direction == 'x_positive':
                    self.X +=1; right_point = self.Y - 1; left_point = self.Y + 1; center_point = self.X
                    print('self.X: '+str(self.X))
                elif turtlebot_direction == 'y_positive':
                    self.Y +=1; right_point = self.X + 1; left_point = self.X - 1; center_point = self.Y
                    print('self.Y: '+str(self.Y))
                elif turtlebot_direction == 'x_negative':
                    self.X -=1; right_point = self.Y + 1; left_point = self.Y - 1; center_point = self.X
                    print('self.X: '+str(self.X))
                elif turtlebot_direction == 'y_negative':
                    self.Y -=1; right_point = self.X - 1; left_point = self.X + 1; center_point = self.Y
                    print('self.Y: '+str(self.Y))
                self.go_straight()
            elif (left_X,left_Y) in self.back_tracking_point and (right_X,right_Y) in self.back_tracking_point:
                print('right_dis - lef_dis: '+str(right_min_distance)+' - '+str(left_min_distance))
                right_distance = sqrt(pow((center_point), 2) + pow((right_point), 2))
                left_distance = sqrt(pow((center_point), 2) + pow((left_point), 2))
                if self.X == 0 or self.Y == 0:
                    if left_min_distance > right_min_distance:
                        print('turn left xy=0')
                        self.turn('left')
                    else:
                        print('turn right xy=0')
                        self.turn('right')
                elif left_distance > right_distance:
                    print('turn left > right')
                    self.turn('left')
                else:
                    print('turn right > left')
                    self.turn('right')
            elif (left_X,left_Y) in self.back_tracking_point:
                print('right_dis - lef_dis: '+str(right_min_distance)+' - '+str(left_min_distance))
                print('turn just left')
                self.turn('left')
            elif (right_X,right_Y) in self.back_tracking_point:
                print('right_dis - lef_dis: '+str(right_min_distance)+' - '+str(left_min_distance))
                print('turn just right')
                self.turn('right')
            else:
                print('right_dis - lef_dis: '+str(right_min_distance)+' - '+str(left_min_distance))
                print('go ptp')
                self.point_to_point()
            
            front_min_distance = min(self.get_scan(0))
            left_min_distance = min(self.get_scan(90))
            right_min_distance = min(self.get_scan(-90))
            turtlebot_direction = self.check_direction()
            print('turtlebot_direction: '+ turtlebot_direction)
            self.add_wall(turtlebot_direction, front_min_distance, left_min_distance, right_min_distance)
            self.add_trajectory()
            self.add_back_tracking()
            print('self.wall: ')
            print(self.wall)
            print('self.trajectory: ')
            print(self.trajectory)
            print('self.back_tracking_point: ')
            print(self.back_tracking_point)
            if len(self.back_tracking_point) == 0:
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0
                self.cmd_vel.publish(self.move_cmd)
            (self.position, self.quat) = self.get_odom()
            print('self.position.x/0.31: '+str(self.position.x/0.31)+'  self.position.y/0.31: '+str(self.position.y/0.31)+'  self.quat_z_and_w: '+str(self.quat.z)+'  '+str(self.quat.w))
            print(' ')

    def add_back_tracking(self):
        surroundings = [(self.X+1,self.Y),
                        (self.X,self.Y+1),
                        (self.X-1,self.Y),
                        (self.X,self.Y-1)]
        for surrounding in surroundings:
            if surrounding not in self.wall:
                if surrounding not in self.trajectory:
                    self.back_tracking_point.append(surrounding)
                    self.back_tracking_point = list(dict.fromkeys(self.back_tracking_point))

    def add_wall(self,direction,front,left,right):
        if front <= SAFE_DISTANCE:
            if direction == 'x_positive':
                self.wall.append((self.X+1,self.Y))
            elif direction == 'y_positive':
                self.wall.append((self.X,self.Y+1))
            elif direction == 'x_negative':
                self.wall.append((self.X-1,self.Y))
            elif direction == 'y_negative':
                self.wall.append((self.X,self.Y-1))
        if left <= SAFE_DISTANCE_SIDE:
            if direction == 'x_positive':
                self.wall.append((self.X,self.Y+1))
            elif direction == 'y_positive':
                self.wall.append((self.X-1,self.Y))
            elif direction == 'x_negative':
                self.wall.append((self.X,self.Y-1))
            elif direction == 'y_negative':
                self.wall.append((self.X+1,self.Y))
        if right <= SAFE_DISTANCE_SIDE:
            if direction == 'x_positive':
                self.wall.append((self.X,self.Y-1))
            elif direction == 'y_positive':
                self.wall.append((self.X+1,self.Y))
            elif direction == 'x_negative':
                self.wall.append((self.X,self.Y+1))
            elif direction == 'y_negative':
                self.wall.append((self.X-1,self.Y))
        self.wall = list(dict.fromkeys(self.wall))

    def add_trajectory(self):
        self.trajectory.append((self.X, self.Y))
        self.trajectory = list(dict.fromkeys(self.trajectory))
        if (self.X,self.Y) in self.back_tracking_point:
            self.back_tracking_point.remove((self.X, self.Y))

    def check_front(self,X,Y,direction):
        if direction == 'x_positive':
            X+=1
        elif direction == 'y_positive':
            Y+=1
        elif direction == 'x_negative':
            X-=1
        elif direction == 'y_negative':
            Y-=1
        return (X,Y)

    def check_left(self,X,Y,direction):
        if direction == 'x_positive':
            Y+=1
        elif direction == 'y_positive':
            X-=1
        elif direction == 'x_negative':
            Y-=1
        elif direction == 'y_negative':
            X+=1
        return (X,Y)

    def check_right(self,X,Y,direction):
        if direction == 'x_positive':
            Y-=1
        elif direction == 'y_positive':
            X+=1
        elif direction == 'x_negative':
            Y+=1
        elif direction == 'y_negative':
            X-=1
        return (X,Y)

    def go_straight(self):
        print('go_traight')
        last_distance = 2*CELL_LENGTH
        goal_x = self.X*CELL_LENGTH
        goal_y = self.Y*CELL_LENGTH
        distance = sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))
        print('distance: '+str(distance))
        while distance > 0.003:
            self.move_cmd.linear.x = LINEAR_VEL
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            self.r.sleep()
            (self.position, no_use) = self.get_odom()
            distance = sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))
            if format(distance,".4f") > format(last_distance,".4f"):
                print('break')
                print('distance vs last_distance: '+str(format(distance,".4f"))+' vs '+str(format(last_distance,".4f")))
                break
            last_distance = distance
        print('distance later: '+str(distance))

    def turn(self,turn_direction):
        turtlebot_direction = self.check_direction()
        if turn_direction == 'left':
            if turtlebot_direction == 'x_positive':
                self.goal_quat_z = QUAT_90DEGREE[2]
                self.turn_by_quat_z()
            elif turtlebot_direction == 'y_positive':
                self.goal_quat_w = 0; self.goal_quat_z = 1
                self.turn_by_quat_w_to_x_negative()
            elif turtlebot_direction == 'x_negative':
                self.goal_quat_w = QUAT_90DEGREE[3]; self.goal_quat_z = - QUAT_90DEGREE[2]
                self.turn_by_quat_w_to_y_negative()
            elif turtlebot_direction == 'y_negative':
                self.goal_quat_z = 0
                self.turn_by_quat_z()
        elif turn_direction == 'right':
            if turtlebot_direction == 'x_positive':
                self.goal_quat_z = - QUAT_90DEGREE[2]
                self.turn_by_quat_z()
            elif turtlebot_direction == 'y_positive':
                self.goal_quat_z = 0
                self.turn_by_quat_z()
            elif turtlebot_direction == 'x_negative':
                self.goal_quat_w = QUAT_90DEGREE[3]; self.goal_quat_z = QUAT_90DEGREE[2]
                self.turn_by_quat_w_to_y_positive()
            elif turtlebot_direction == 'y_negative':
                self.goal_quat_w = 0; self.goal_quat_z = 1
                self.turn_by_quat_w_to_x_negative()
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)

    def turn_by_quat_z(self):
        (self.position, self.quat) = self.get_odom()
        while abs(self.goal_quat_z - self.quat.z) > 0.001:
            if self.quat.z < self.goal_quat_z:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = OMEGA_0 + KA1*abs(self.goal_quat_z - self.quat.z)
            else:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = -(OMEGA_0 + KA1*abs(self.goal_quat_z - self.quat.z))
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            (self.position, self.quat) = self.get_odom()

    def turn_by_quat_w_to_x_negative(self):
        (self.position, self.quat) = self.get_odom()
        while abs(self.goal_quat_w - self.quat.w) > 0.001:
            if self.quat.z > 0:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = OMEGA_0 + KA2*abs(self.goal_quat_w - self.quat.w)
            else:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = -(OMEGA_0 + KA2*abs(self.goal_quat_w - self.quat.w))
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            (self.position, self.quat) = self.get_odom()

    def turn_by_quat_w_to_y_negative(self):
        (self.position, self.quat) = self.get_odom()
        while abs(self.goal_quat_w - self.quat.w) > 0.001:
            if self.quat.z < - QUAT_90DEGREE[2] or self.quat.z > QUAT_90DEGREE[2]:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = OMEGA_0 + KA2*abs(self.goal_quat_w - self.quat.w)
            else:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = -(OMEGA_0 + KA2*abs(self.goal_quat_w - self.quat.w))
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            (self.position, self.quat) = self.get_odom()

    def turn_by_quat_w_to_y_positive(self):
        (self.position, self.quat) = self.get_odom()
        while abs(self.goal_quat_w - self.quat.w) > 0.001:
            if self.quat.z < - QUAT_90DEGREE[2] or self.quat.z > QUAT_90DEGREE[2]:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = -(OMEGA_0 + KA2*abs(self.goal_quat_w - self.quat.w))
            else:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = OMEGA_0 + KA2*abs(self.goal_quat_w - self.quat.w)
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            (self.position, self.quat) = self.get_odom()

    def point_to_point(self):
        print('COMMING SOON')

    def check_direction(self):
        if self.goal_quat_z == 0:
            direction = 'x_positive'
        elif self.goal_quat_z == QUAT_90DEGREE[2]:
            direction = 'y_positive'
        elif self.goal_quat_z == 1:
            direction = 'x_negative'
        elif self.goal_quat_z == - QUAT_90DEGREE[2]:
            direction = 'y_negative'
        return direction

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
            (roll,pitch,yaw) = euler_from_quaternion(rot)
            rot = quaternion_from_euler(roll,pitch,yaw)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), Quaternion(*rot))

if __name__ == '__main__':
    rospy.init_node('TASP4', anonymous=False)
    try:
        print('Start')
        GoAndTurn()
    except: rospy.loginfo("Shutdown program.")