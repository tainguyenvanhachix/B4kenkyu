#!/usr/bin/env python3
import math
import rospy
import tf
from math import atan2, pi, pow, sqrt, degrees, radians, cos
from geometry_msgs.msg import Point, Quaternion, Twist
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from setting import *

class GoAndTurn():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel',Twist,queue_size=10)
        self.trajectory_line_maker = rospy.Publisher('trajectory_marker', MarkerArray, queue_size=1)

        # The starting point
        self.X = 0
        self.Y = 0

        self.r1 = rospy.Rate(20)
        self.r2 = rospy.Rate(8)

        self.position = Point()
        self.move_cmd = Twist()
        self.sum_marker = MarkerArray()
        self.marker1 = Marker()
        self.sum_marker.markers = [self.marker1]

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(1.0))

        # Setting of wall, trajectory, backtracking point at the beginging
        self.targets =[(2,0),(2,-1),(0,-1),(0,3),(4,3),(4,-1),(4,1),(3,1),(3,2),(2,2),(2,0),(0,0)]

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

        (self.position, self.quat,no_use) = self.get_odom()
        self.check_around_and_go()

    def check_around_and_go(self):
        for target in self.targets:
            (goal_x,goal_y) = target
            goal_angle = atan2(goal_y*CELL_LENGTH - self.position.y, goal_x*CELL_LENGTH - self.position.x)
            
            if target == (4,1):
                self.back(goal_x,goal_y)
            else:
                self.turn_to_goal(goal_angle)
                self.go_straight_to_goal(goal_x,goal_y)

            # Publish the trajectory marker and delete BTP marker
            self.line_point = Point()
            self.line_point.x = self.X*CELL_LENGTH
            self.line_point.y = self.Y*CELL_LENGTH
            self.line_point.z = 0.0
            self.marker1.points.append(self.line_point)
            self.trajectory_line_maker.publish(self.sum_marker)
        self.turn_to_goal(0)

    def turn_to_goal(self,goal_angle):
        (self.position, self.quat, present_angle) = self.get_odom()
        print('goal angle - present angle: '+str(goal_angle)+'  '+str(present_angle))
        while abs(goal_angle - present_angle) > 0.01:
            if goal_angle*present_angle < 0 and goal_angle < 0:
                if abs(goal_angle - present_angle) <= pi:
                    self.move_cmd.linear.x = 0
                    self.move_cmd.angular.z = -(OMEGA_0 + KA2*abs(goal_angle - present_angle))
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = OMEGA_0 + KA2*abs(goal_angle + present_angle)
            elif goal_angle*present_angle < 0 and goal_angle > 0:
                if abs(goal_angle -present_angle) <= pi:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = OMEGA_0 + KA2*abs(goal_angle - present_angle)
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -(OMEGA_0 + KA2*abs(goal_angle + present_angle))
            else:
                if goal_angle > present_angle:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = OMEGA_0 + KA2*abs(goal_angle - present_angle)
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -(OMEGA_0 + KA2*abs(goal_angle - present_angle))
            self.cmd_vel.publish(self.move_cmd)
            self.r1.sleep()
            (self.position, self.quat, present_angle) = self.get_odom()
        print('done turn')
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)

    def back(self,goal_x,goal_y):
        print('back')
        self.X = goal_x
        self.Y = goal_y
        goal_x = CELL_LENGTH*goal_x
        goal_y = CELL_LENGTH*goal_y
        distance = sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))
        last_distance = distance + 1
        print('distance: '+str(distance))
        while distance > 0.025:
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

    def go_straight_to_goal(self,goal_x,goal_y):
        print('go_traight to goal')
        self.X = goal_x
        self.Y = goal_y
        goal_x = CELL_LENGTH*goal_x
        goal_y = CELL_LENGTH*goal_y
        distance = sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))
        last_distance = distance + 1
        print('distance: '+str(distance))
        while distance > 0.025:
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

    def get_odom(self):
            try:
                (trans, rot) = self.tf_listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
                (roll,pitch,yaw) = euler_from_quaternion(rot)
                rot = quaternion_from_euler(roll,pitch,yaw)
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("TF Exception")
                return
            return (Point(*trans), Quaternion(*rot), yaw)

if __name__ == '__main__':
    rospy.init_node('TASP', anonymous=False)
    try:
        print('Start')
        GoAndTurn()
    except: rospy.loginfo("Shutdown program.")