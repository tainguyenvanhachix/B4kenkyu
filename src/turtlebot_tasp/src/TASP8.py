#!/usr/bin/env python3
import math
import rospy
import tf
from math import atan2, pi, pow, sqrt
from geometry_msgs.msg import Point, Quaternion, Twist
from turtlebot_tasp.msg import mapdataPoint
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from setting import *

class GoAndTurn():
    def __init__(self):
        # Declares that node is publishing to the cmd_vel topic
        self.cmd_vel = rospy.Publisher('cmd_vel',Twist,queue_size=10)
        self.trajectory_line_maker = rospy.Publisher('trajectory_marker', MarkerArray, queue_size=1)
        self.pub_wall = rospy.Publisher('wall',mapdataPoint,queue_size=1000)
        self.pub_BTP = rospy.Publisher('BTP',mapdataPoint,queue_size=1000)

        # The starting point
        self.X = 0
        self.Y = 0
        self.goal_quat_z = 0
        self.goal_quat_w = 1

        # wall and BTP data for map
        self.wall_point = mapdataPoint()
        self.BTP_point = mapdataPoint()
        self.r1 = rospy.Rate(20)
        self.r2 = rospy.Rate(8)

        # Setting of wall, trajectory, backtracking point at the beginging
        self.trajectory = []
        # self.wall =[(-1,0),(-1,1),(-1,2),(0,-1),(1,-1),(2,0),(2,1),(3,1),(4,2),(3,3),(2,3),(1,3),(0,3)] #1
        # self.wall =[(-1,0),(-1,1),(0,-1),(1,-1),(1,-2),(2,-3),(3,-2),(3,-1),(3,0),(3,1),(2,2),(1,2),(0,2)] #2
        self.wall =[(-1,0),(-1,1),(-1,2),(0,-1),(1,-1),(2,-1),(3,-1),(4,0),(4,1),(4,2),(4,3),(3,4),(2,4),(1,4),(0,3)] #3

        self.back_tracking_point = [(0,0)]
        self.BTP_point = mapdataPoint()
        (self.BTP_point.x,self.BTP_point.y) = (0,0)
        self.r2.sleep()
        self.pub_BTP.publish(self.BTP_point)

        self.position = Point()
        self.quat = Quaternion()
        self.move_cmd = Twist()
        self.sum_marker = MarkerArray()
        self.marker1 = Marker()
        self.marker2 = Marker()
        self.sum_marker.markers = [self.marker1,self.marker2]

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(1.0))

        # Maker1 init
        self.marker1.header.frame_id = "map"
        self.marker1.type = self.marker1.LINE_STRIP
        self.marker1.action = self.marker1.ADD
        self.marker1.id = 1
        # self.marker1 scale
        self.marker1.scale.x = 0.03
        self.marker1.scale.y = 0.0
        self.marker1.scale.z = 0.0
        # self.marker1 color
        self.marker1.color.a = 1.0
        self.marker1.color.r = 1.0
        self.marker1.color.g = 0.0
        self.marker1.color.b = 0.0
        # self.marker1 orientaiton
        self.marker1.pose.orientation.x = 0.0
        self.marker1.pose.orientation.y = 0.0
        self.marker1.pose.orientation.z = 0.0
        self.marker1.pose.orientation.w = 1.0
        # self.marker1 position
        self.marker1.pose.position.x = 0.0
        self.marker1.pose.position.y = 0.0
        self.marker1.pose.position.z = 0.0
        # self.marker1 line points
        self.marker1.points = []
        # first point
        self.line_point = Point()
        self.line_point.x = 0.0
        self.line_point.y = 0.0
        self.line_point.z = 0.0
        self.marker1.points.append(self.line_point)

        # Maker2 init
        self.marker2.header.frame_id = "map"
        self.marker2.type = self.marker2.SPHERE_LIST
        self.marker2.action = self.marker2.ADD
        self.marker2.id = 2
        # self.marker2 scale
        self.marker2.scale.x = 0.05
        self.marker2.scale.y = 0.05
        self.marker2.scale.z = 0.05
        # self.marker2 color
        self.marker2.color.a = 1.0
        self.marker2.color.r = 1.0
        self.marker2.color.g = 1.0
        self.marker2.color.b = 0.0
        # self.marker2 orientaiton
        self.marker2.pose.orientation.x = 0.0
        self.marker2.pose.orientation.y = 0.0
        self.marker2.pose.orientation.z = 0.0
        self.marker2.pose.orientation.w = 1.0
        # self.marker2 position
        self.marker2.pose.position.x = 0.0
        self.marker2.pose.position.y = 0.0
        self.marker2.pose.position.z = 0.0
        # self.marker2 line points
        self.marker2.points = []
        # start point
        self.sphere_point = Point()
        self.sphere_point.x = 0.0
        self.sphere_point.y = 0.0
        self.sphere_point.z = 0.0
        self.marker2.points.append(self.sphere_point)

        (self.position, self.quat,no_use) = self.get_odom()
        self.check_around_and_go()
# 0-----------------------------------------------------------------

    def check_around_and_go(self):
        # variable to check already back to start or not
        back_to_start = True

        # Check if obstacle behind robot at the beginning
        if min(self.get_scan(-180)) < SAFE_DISTANCE_BEHIND:
            self.wall.append((self.X-1,self.Y))
            self.publish_wall()
        # Publish the Marker
        self.trajectory_line_maker.publish(self.sum_marker)
        
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
        print(' ')

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
                self.go_straight(turtlebot_direction)
            # Check if left and right in backtracking point, turn
            elif (left_X,left_Y) in self.back_tracking_point and (right_X,right_Y) in self.back_tracking_point:
                print('right_dis - lef_dis: '+str(right_min_distance)+' - '+str(left_min_distance))
                right_distance = sqrt(pow((center_point), 2) + pow((right_point), 2))
                left_distance = sqrt(pow((center_point), 2) + pow((left_point), 2))

                # The same when compare with starting point
                if self.X == 0 or self.Y == 0:
                    if left_min_distance > right_min_distance:
                        print('turn left xy=0')
                        self.turn('left')
                    else:
                        print('turn right xy=0')
                        self.turn('right')
                # Turn away from starting point
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
            
            # Check if obstacle front-left-right to assign wall, update trajectory and backtraking point
            front_min_distance = min(self.get_scan(0))
            left_min_distance = min(self.get_scan(90))
            right_min_distance = min(self.get_scan(-90))
            turtlebot_direction = self.check_direction()
            print('turtlebot_direction: '+ turtlebot_direction)
            self.add_wall(turtlebot_direction, front_min_distance, left_min_distance, right_min_distance)
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

                if back_to_start:
                    self.back_tracking_point.append((0,0))
                    back_to_start = False

            # Publish the trajectory marker and delete BTP marker
            self.line_point = Point()
            self.line_point.x = self.X*CELL_LENGTH
            self.line_point.y = self.Y*CELL_LENGTH
            self.line_point.z = 0.0
            self.marker1.points.append(self.line_point)
            for sphere_point in self.marker2.points:
                if sphere_point.x == self.X*CELL_LENGTH and sphere_point.y == self.Y*CELL_LENGTH:
                    self.marker2.points.remove(sphere_point)
            self.trajectory_line_maker.publish(self.sum_marker)
            
            # Print present position
            (self.position, self.quat,no_use) = self.get_odom()
            print('self.position.x/0.31: '+str(self.position.x/0.31)+'  self.position.y/0.31: '+str(self.position.y/0.31)+'  self.quat_z_and_w: '+str(self.quat.z)+'  '+str(self.quat.w))
            print(' ')

        # Turn to starting rotation
        self.turn_to_goal(0)
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)
        (self.position, self.quat,no_use) = self.get_odom()
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
                    self.publish_BTP()
                    self.back_tracking_point = list(dict.fromkeys(self.back_tracking_point))
                    # Add BTP marker
                    self.sphere_point = Point()
                    self.sphere_point.x = surrounding[0]*CELL_LENGTH
                    self.sphere_point.y = surrounding[1]*CELL_LENGTH
                    self.sphere_point.z = 0.0
                    if self.sphere_point not in self.marker2.points:
                        self.marker2.points.append(self.sphere_point)

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
            self.publish_wall()
        if left <= SAFE_DISTANCE_FOOTBASE:
            if direction == 'x_positive':
                self.wall.append((self.X,self.Y+1))
            elif direction == 'y_positive':
                self.wall.append((self.X-1,self.Y))
            elif direction == 'x_negative':
                self.wall.append((self.X,self.Y-1))
            elif direction == 'y_negative':
                self.wall.append((self.X+1,self.Y))
            self.publish_wall()
        if right <= SAFE_DISTANCE_FOOTBASE:
            if direction == 'x_positive':
                self.wall.append((self.X,self.Y-1))
            elif direction == 'y_positive':
                self.wall.append((self.X+1,self.Y))
            elif direction == 'x_negative':
                self.wall.append((self.X,self.Y+1))
            elif direction == 'y_negative':
                self.wall.append((self.X-1,self.Y))
            self.publish_wall()
        self.wall = list(dict.fromkeys(self.wall))

    def add_trajectory(self):
        self.trajectory.append((self.X, self.Y))
        # self.trajectory = list(dict.fromkeys(self.trajectory))
        if (self.X,self.Y) in self.back_tracking_point:
            self.back_tracking_point.remove((self.X, self.Y))
# 2-----------------------------------------------------------------
    # Publish added wall
    def publish_wall(self):
        self.wall_point = mapdataPoint()
        (self.wall_point.x,self.wall_point.y) = self.wall[-1]
        self.pub_wall.publish(self.wall_point)
        print(self.wall_point)

    # Pulish added BTP
    def publish_BTP(self):
        self.BTP_point = mapdataPoint()
        (self.BTP_point.x,self.BTP_point.y) = self.back_tracking_point[-1]
        self.pub_BTP.publish(self.BTP_point)
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

    def go_straight(self,turtlebot_direction):
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
            if (format(distance,".4f") > format(last_distance,".4f")) and (distance < 0.2):
                print('break')
                print('distance vs last_distance: '+str(format(distance,".4f"))+' vs '+str(format(last_distance,".4f")))
                break
            last_distance = distance

        # Stop right away if cannot continue go straight
        (front_X,front_Y) = self.check_front(self.X, self.Y,turtlebot_direction)
        if (min(self.get_scan(0)) < SAFE_DISTANCE_FRONT) or ((front_X,front_Y) in self.wall):
            print('stop to turn')
            self.move_cmd.linear.x = 0
            self.cmd_vel.publish(self.move_cmd)

        self.add_trajectory()
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
        min_distance = 0
        goal_angle = 0
        goal_angle_real = 0
        directly = False
        (self.position, self.quat, present_angle) = self.get_odom()

        # Check every BTP if can go directly
        for backtracking_point in self.back_tracking_point:
            if self.check_go_directly(self.X, self.Y, backtracking_point):
                # Assign nearest BTP
                (goal_x,goal_y,min_distance,goal_angle,goal_angle_real) = self.nearest_BTP(backtracking_point, min_distance, goal_angle, goal_angle_real, present_angle)

                directly = True

        # If can not go directly, go to intermediary point
        if directly == False:
            # Assign goal to the nearest BTP
            for backtracking_point in self.back_tracking_point:
                print('check nearest with cannot directly')
                (goal_x,goal_y,min_distance,goal_angle,goal_angle_real) = self.nearest_BTP(backtracking_point, min_distance, goal_angle, goal_angle_real, present_angle)

            # Find intermediary point
            (goal_x, goal_y, goal_angle, goal_angle_real) = self.find_intermediary(goal_x, goal_y)

        print('point to point goal: '+str(goal_x)+' '+str(goal_y)+' goal_angle: '+str(goal_angle)+' min distance: '+str(min_distance))

        # Back if BTP is behind or keep going straght if BTP is front
        if (goal_x == self.X or goal_y == self.Y) and (abs(goal_angle - present_angle) < 0.02 or abs(abs(goal_angle - present_angle) - pi) < 0.02):
            if abs(goal_angle - present_angle) < 0.02:
                self.go_straight_to_goal(goal_x,goal_y)
            elif abs(abs(goal_angle - present_angle) - pi) < 0.02:
                self.back(goal_x, goal_y)
        else:
            self.turn_to_goal(goal_angle_real)
            self.go_straight_to_goal(goal_x,goal_y)
        # Check BTP arround and turn to that point
        # Turn to next BTP if no BTP around
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)

    def check_go_directly(self, x1, y1, point):
        (x2, y2) = point
        check = True

        # Find line equation from two point, ax + by + c = 0  
        if x1 == x2:
            a = 1; b = 0; c = -x1
        elif y1 == y2:
            a = 0; b = 1; c = -y1
        else:
            a = x2-x1; b = (y1-y2)/(x2-x1); c = x1-y1

        # Line segment quare limitation
        if x1 >= x2:
            x_max = x1; x_min = x2
        else:
            x_max = x2; x_min = x1
        if y1 >= y2:
            y_max = y1; y_min = y2
        else:
            y_max = y2; y_min = y1
        
        # Check in line segment
        for wall in self.wall:
            if x_max >= wall[0] and wall[0] >= x_min and y_max >= wall[1] and wall[1] >= y_min:
                distance_center = abs(a*wall[0] + b*wall[1] + c)/sqrt(pow(a,2)+pow(b,2))
                distance_top_right    = abs(a*(wall[0]-0.5) + b*(wall[1]-0.5) + c)/sqrt(pow(a,2)+pow(b,2))
                distance_top_left     = abs(a*(wall[0]-0.5) + b*(wall[1]+0.5) + c)/sqrt(pow(a,2)+pow(b,2))
                distance_bottom_right = abs(a*(wall[0]+0.5) + b*(wall[1]-0.5) + c)/sqrt(pow(a,2)+pow(b,2))
                distance_bottom_left  = abs(a*(wall[0]+0.5) + b*(wall[1]+0.5) + c)/sqrt(pow(a,2)+pow(b,2))
                if distance_center < 1 or distance_top_right < 0.5 or distance_top_left < 0.5 or distance_bottom_right < 0.5 or distance_bottom_left < 0.5:
                    check = False
        return check

    def nearest_BTP(self, backtracking_point, min_distance, goal_angle, goal_angle_real, present_angle):
        distance = sqrt(pow((backtracking_point[0] - self.X), 2) + pow((backtracking_point[1] - self.Y), 2))

        # Assign goal to BTP has shorter than min_distance
        if min_distance == 0 or distance < min_distance:
            (goal_x,goal_y) = backtracking_point
            min_distance =  distance
            goal_angle = atan2(goal_y - self.Y, goal_x - self.X)
            goal_angle_real = atan2(backtracking_point[1]*CELL_LENGTH - self.position.y, backtracking_point[0]*CELL_LENGTH - self.position.x)
        # If the same distance, compare less rotation
        elif distance == min_distance:
            angle = atan2(backtracking_point[1] - self.Y, backtracking_point[0] - self.X)
            if abs(angle-present_angle) < abs(goal_angle-present_angle):
                min_distance = distance
                (goal_x,goal_y) = backtracking_point
                goal_angle = angle
                goal_angle_real = atan2(backtracking_point[1]*CELL_LENGTH - self.position.y, backtracking_point[0]*CELL_LENGTH - self.position.x)

        return (goal_x,goal_y,min_distance,goal_angle,goal_angle_real)

    def find_intermediary(self, goal_x, goal_y):
        print('find intermediary')
        min_distance = 0
        for intermediary in self.trajectory:
            if self.check_go_directly(self.X, self.Y, intermediary) and self.check_go_directly(goal_x, goal_y, intermediary):
                distance = sqrt(pow((intermediary[0] - self.X), 2) + pow((intermediary[1] - self.Y), 2)) + sqrt(pow((intermediary[0] - goal_x), 2) + pow((intermediary[1] - goal_y), 2))
                if min_distance == 0 or distance < min_distance:
                    min_distance = distance
                    (intermediary_goal_x, intermediary_goal_y) = (intermediary)
                    goal_angle = atan2(intermediary_goal_y - self.Y, intermediary_goal_x - self.X)
                    goal_angle_real = atan2(intermediary_goal_y*CELL_LENGTH - self.position.y, intermediary_goal_x*CELL_LENGTH - self.position.x)
        return (intermediary_goal_x, intermediary_goal_y, goal_angle, goal_angle_real)

    def back(self,goal_x,goal_y):
        print('back')
        self.X = goal_x
        self.Y = goal_y
        goal_x = CELL_LENGTH*goal_x
        goal_y = CELL_LENGTH*goal_y
        distance = sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))
        last_distance = distance + 1
        print('distance: '+str(distance))
        while distance > 0.003:
            self.move_cmd.linear.x = -LINEAR_VEL
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
            self.r2.sleep()
            (self.position, no_use, no_use) = self.get_odom()
            distance = sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))
            if (format(distance,".4f") > format(last_distance,".4f")) and (distance < 0.2):
                print('break')
                print('distance vs last_distance: '+str(format(distance,".4f"))+' vs '+str(format(last_distance,".4f")))
                break
            last_distance = distance

        # Stop after finish PTP
        self.move_cmd.linear.x = 0
        self.cmd_vel.publish(self.move_cmd)
        print('distance later: '+str(distance))

        # Add trajectory
        self.add_trajectory()

    def turn_to_goal(self,goal_angle):
        (self.position, self.quat, present_angle) = self.get_odom()
        print('goal angle - present angle: '+str(goal_angle)+'  '+str(present_angle))
        while abs(goal_angle - present_angle) > 0.01:
            if goal_angle*present_angle < 0 and goal_angle < 0:
                if abs(goal_angle - present_angle) <= pi:
                    self.move_cmd.angular.z = -(OMEGA_0 + KA2*abs(goal_angle - present_angle))
                else:
                    self.move_cmd.angular.z = OMEGA_0 + KA2*abs(goal_angle - present_angle)
            elif goal_angle*present_angle < 0 and goal_angle > 0:
                if abs(goal_angle -present_angle) <= pi:
                    self.move_cmd.angular.z = OMEGA_0 + KA2*abs(goal_angle - present_angle)
                else:
                    self.move_cmd.angular.z = -(OMEGA_0 + KA2*abs(goal_angle - present_angle))
            else:
                if goal_angle > present_angle:
                    self.move_cmd.angular.z = OMEGA_0 + KA2*abs(goal_angle - present_angle)
                else:
                    self.move_cmd.angular.z = -(OMEGA_0 + KA2*abs(goal_angle - present_angle))
            self.cmd_vel.publish(self.move_cmd)
            self.r1.sleep()
            (self.position, self.quat,present_angle) = self.get_odom()
        print('done')

    def go_straight_to_goal(self,goal_x,goal_y):
        print('go_traight')
        self.X = goal_x
        self.Y = goal_y
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
            if (format(distance,".4f") > format(last_distance,".4f")) and (distance < 0.2):
                print('break')
                print('distance vs last_distance: '+str(format(distance,".4f"))+' vs '+str(format(last_distance,".4f")))
                break
            last_distance = distance

        # Stop after finish PTP
        self.move_cmd.linear.x = 0
        self.cmd_vel.publish(self.move_cmd)
        print('distance later: '+str(distance))

        # Add trajectory
        self.add_trajectory()
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
    rospy.init_node('TASP', anonymous=False)
    try:
        print('Start')
        GoAndTurn()
    except: rospy.loginfo("Shutdown program.")