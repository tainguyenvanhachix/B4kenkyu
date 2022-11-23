#!/usr/bin/env python3
import math
import rospy
import tf
from math import atan2, pi, pow, sqrt
from geometry_msgs.msg import Point, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from setting import *

class GoAndTurn():
    def __init__(self):
        # Declares that node is publishing to the cmd_vel topic
        self.cmd_vel = rospy.Publisher('cmd_vel',Twist,queue_size=5)

        # The starting point
        self.X = 0
        self.Y = 0
        self.goal_quat_z = 0
        self.goal_quat_w = 1

        # Setting of wall, trajectory, backtracking point at the beginging
        self.trajectory = []
        self.wall =[(-1,0),(-1,1),(-1,2),(0,-1),(1,-1),(2,-1),(3,0),(3,1),(3,2),(0,3),(1,3),(2,3)]
        self.back_tracking_point = [(0,0)]

        self.position = Point()
        self.quat = Quaternion()
        self.move_cmd = Twist()

        self.r1 = rospy.Rate(20)
        self.r2 = rospy.Rate(8)

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(1.0))

        (self.position, self.quat,no_use) = self.get_odom()
        self.check_around_and_go()
# 0-----------------------------------------------------------------

    def check_around_and_go(self):
        # Check if obstacle behind robot at the beginning
        if min(self.get_scan(-180)) < SAFE_DISTANCE_BEHIND:
            self.wall.append((self.X-1,self.Y))
        
        # Check if obstacle front-left-right to assign wall, trajectory and backtraking point at the beginning
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

        # If backtracking point still exist, move to it
        while len(self.back_tracking_point) > 0:
            # Assign position around robot
            (front_X,front_Y) = self.check_front(self.X,self.Y,turtlebot_direction)
            (left_X,left_Y) = self.check_left(self.X,self.Y,turtlebot_direction)
            (right_X,right_Y) = self.check_right(self.X,self.Y,turtlebot_direction)

            # Check if front in backtracking point, go straight
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
            # Check if left and right in backtracking point, turn
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
                print('lef_dis: '+str(left_min_distance))
                print('turn just left')
                self.turn('left')
            elif (right_X,right_Y) in self.back_tracking_point:
                print('right_dis: '+str(right_min_distance))
                print('turn just right')
                self.turn('right')
            else:
                print('go ptp')
                self.point_to_point()
            
            # Check if obstacle front-left-right to assign wall, trajectory and backtraking point
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

            # Stop if no backtracking point
            if len(self.back_tracking_point) == 0:
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0
                self.cmd_vel.publish(self.move_cmd)
            
            # Print presentt position
            (self.position, self.quat,no_use) = self.get_odom()
            print('self.position.x/0.31: '+str(self.position.x/0.31)+'  self.position.y/0.31: '+str(self.position.y/0.31)+'  self.quat_z_and_w: '+str(self.quat.z)+'  '+str(self.quat.w))
            print(' ')

        self.go_to_starting_point()
        print('self.position.x/0.31: '+str(self.position.x/0.31)+'  self.position.y/0.31: '+str(self.position.y/0.31)+'  self.quat_z_and_w: '+str(self.quat.z)+'  '+str(self.quat.w))
# 1-----------------------------------------------------------------

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
        if front <= SAFE_DISTANCE_FRONT:
            if direction == 'x_positive':
                self.wall.append((self.X+1,self.Y))
            elif direction == 'y_positive':
                self.wall.append((self.X,self.Y+1))
            elif direction == 'x_negative':
                self.wall.append((self.X-1,self.Y))
            elif direction == 'y_negative':
                self.wall.append((self.X,self.Y-1))
        if left <= SAFE_DISTANCE_FOOTBASE:
            if direction == 'x_positive':
                self.wall.append((self.X,self.Y+1))
            elif direction == 'y_positive':
                self.wall.append((self.X-1,self.Y))
            elif direction == 'x_negative':
                self.wall.append((self.X,self.Y-1))
            elif direction == 'y_negative':
                self.wall.append((self.X+1,self.Y))
        if right <= SAFE_DISTANCE_FOOTBASE:
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
# 2-----------------------------------------------------------------

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
# 3-----------------------------------------------------------------

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
            self.r2.sleep()
            (self.position, no_use, no_use) = self.get_odom()
            distance = sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))
            if format(distance,".4f") > format(last_distance,".4f"):
                print('break')
                print('distance vs last_distance: '+str(format(distance,".4f"))+' vs '+str(format(last_distance,".4f")))
                break
            last_distance = distance
        # Stop right away if cannot continue go straight
        if min(self.get_scan(0)) < SAFE_DISTANCE_FRONT: # need to add wall also -> comming soon
            self.move_cmd.linear.x = 0
            self.cmd_vel.publish(self.move_cmd)
        print('distance later: '+str(distance))
# 4-----------------------------------------------------------------

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
        (self.position, self.quat,no_use) = self.get_odom()
        while abs(self.goal_quat_z - self.quat.z) > 0.001:
            if self.quat.z < self.goal_quat_z:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = OMEGA_0 + KA1*abs(self.goal_quat_z - self.quat.z)
            else:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = -(OMEGA_0 + KA1*abs(self.goal_quat_z - self.quat.z))
            self.cmd_vel.publish(self.move_cmd)
            self.r1.sleep()
            (self.position, self.quat,no_use) = self.get_odom()

    def turn_by_quat_w_to_x_negative(self):
        (self.position, self.quat,no_use) = self.get_odom()
        while abs(self.goal_quat_w - self.quat.w) > 0.001:
            if self.quat.z > 0:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = OMEGA_0 + KA1*abs(self.goal_quat_w - self.quat.w)
            else:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = -(OMEGA_0 + KA1*abs(self.goal_quat_w - self.quat.w))
            self.cmd_vel.publish(self.move_cmd)
            self.r1.sleep()
            (self.position, self.quat,no_use) = self.get_odom()

    def turn_by_quat_w_to_y_negative(self):
        (self.position, self.quat,no_use) = self.get_odom()
        while abs(self.goal_quat_w - self.quat.w) > 0.001:
            if self.quat.z < - QUAT_90DEGREE[2] or self.quat.z > QUAT_90DEGREE[2]:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = OMEGA_0 + KA1*abs(self.goal_quat_w - self.quat.w)
            else:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = -(OMEGA_0 + KA1*abs(self.goal_quat_w - self.quat.w))
            self.cmd_vel.publish(self.move_cmd)
            self.r1.sleep()
            (self.position, self.quat,no_use) = self.get_odom()

    def turn_by_quat_w_to_y_positive(self):
        (self.position, self.quat,no_use) = self.get_odom()
        while abs(self.goal_quat_w - self.quat.w) > 0.001:
            if self.quat.z < - QUAT_90DEGREE[2] or self.quat.z > QUAT_90DEGREE[2]:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = -(OMEGA_0 + KA1*abs(self.goal_quat_w - self.quat.w))
            else:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = OMEGA_0 + KA1*abs(self.goal_quat_w - self.quat.w)
            self.cmd_vel.publish(self.move_cmd)
            self.r1.sleep()
            (self.position, self.quat,no_use) = self.get_odom()
# 5-----------------------------------------------------------------

    def point_to_point(self):
        backtracking_point = self.back_tracking_point[0]
        (goal_x,goal_y) = backtracking_point
        min_distance =  sqrt(pow((backtracking_point[0] - self.X), 2) + pow((backtracking_point[1] - self.Y), 2))
        goal_angle = atan2(backtracking_point[1] - self.Y, backtracking_point[0] - self.X)
        for backtracking_point in self.back_tracking_point:
            distance = sqrt(pow((backtracking_point[0] - self.X), 2) + pow((backtracking_point[1] - self.Y), 2))
            if distance == min_distance:
                angle = atan2(backtracking_point[1] - self.Y, backtracking_point[0] - self.X)
                if abs(angle) < abs(goal_angle): # compare with direction abs(angle-direction) < abs(goal_angle-direction)
                    min_distance = distance
                    (goal_x,goal_y) = backtracking_point
                    goal_angle = angle
            elif distance < min_distance:
                min_distance = distance
                (goal_x,goal_y) = backtracking_point
                goal_angle = atan2(goal_y - self.Y, goal_x - self.X)
        print('point to point goal: '+str(goal_x)+' '+str(goal_y)+' goal_angle: '+str(goal_angle)+' min distance: '+str(min_distance))
        self.turn_to_goal(goal_angle)
        self.go_straight_to_goal(goal_x,goal_y)
        self.X = goal_x
        self.Y = goal_y
        #turn to next BTP
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)

    def turn_to_goal(self,goal_angle):
        (self.position, self.quat, present_angle) = self.get_odom()
        print('goal angle - present angle: '+str(goal_angle)+'  '+str(present_angle))
        if abs(goal_angle - present_angle) <= pi:
            while abs(goal_angle - present_angle) > 0.01:
                if goal_angle - present_angle > 0:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = OMEGA_0 + KA1*abs(goal_angle - present_angle)
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -(OMEGA_0 + KA1*abs(goal_angle - present_angle))
                self.cmd_vel.publish(self.move_cmd)
                self.r1.sleep()
                (self.position, self.quat,present_angle) = self.get_odom()
            print('done')
        else:
            while abs(goal_angle - present_angle) > 0.01:
                if abs(goal_angle - present_angle) > pi:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -ANGULAR_VEL
                else:
                    if goal_angle - present_angle > 0:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = OMEGA_0 + KA1*abs(goal_angle - present_angle)
                    else:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = -(OMEGA_0 + KA1*abs(goal_angle - present_angle))
                self.cmd_vel.publish(self.move_cmd)
                self.r1.sleep()
                (self.position, self.quat,present_angle) = self.get_odom()
            print('done')

    def go_straight_to_goal(self,goal_x,goal_y):
        print('go_traight')
        goal_x = CELL_LENGTH*goal_x
        goal_y = CELL_LENGTH*goal_y
        distance = sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))
        last_distance = distance + 1
        print('distance: '+str(distance))
        while distance > 0.003:
            self.move_cmd.linear.x = LINEAR_VEL
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
            self.r2.sleep()
            (self.position, no_use, no_use) = self.get_odom()
            distance = sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))
            if format(distance,".4f") > format(last_distance,".4f"):
                print('break')
                print('distance vs last_distance: '+str(format(distance,".4f"))+' vs '+str(format(last_distance,".4f")))
                break
            last_distance = distance
        if min(self.get_scan(0)) < SAFE_DISTANCE_FRONT:
            self.move_cmd.linear.x = 0
            self.cmd_vel.publish(self.move_cmd)
        print('distance later: '+str(distance))
# 6-----------------------------------------------------------------

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
# 7-----------------------------------------------------------------

    def go_to_starting_point(self):
        (goal_x,goal_y) = (0,0)
        goal_angle = atan2(0 - self.Y, 0 - self.X)
        distance = sqrt(pow((0 - self.X), 2) + pow((0 - self.Y), 2))
        print('point to point goal: '+str(goal_x)+' '+str(goal_y)+' goal_angle: '+str(goal_angle)+'distance'+str(distance))
        self.turn_to_goal(goal_angle)
        self.go_straight_to_goal(goal_x,goal_y)
        self.turn_to_goal(0)
        self.X = goal_x
        self.Y = goal_y

        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)
# 8-----------------------------------------------------------------

    # Get scan_filter array that contains distance from lidar to obstacle
    def get_scan(self, scan_direction):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
        scan_filter_after = []
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
            if scan_filter[i] < 0.1:
                pass
            elif math.isnan(scan_filter[i]):
                pass
            else:
                scan_filter_after.append(scan_filter[i])
        return scan_filter_after

    # Get self.position and rotation of turtlebot base on different bettween odom and base_footprint frame
    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
            (roll,pitch,yaw) = euler_from_quaternion(rot)
            rot = quaternion_from_euler(roll,pitch,yaw)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), Quaternion(*rot), yaw)
# 8-----------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('TASP4', anonymous=False)
    try:
        print('Start')
        GoAndTurn()
    except: rospy.loginfo("Shutdown program.")