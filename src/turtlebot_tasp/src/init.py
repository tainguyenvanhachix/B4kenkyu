#!/usr/bin/env python3
import math
import rospy
import tf
from math import atan2, pi, pow, sqrt, degrees, radians, cos
from geometry_msgs.msg import Point, Quaternion, Twist
from turtlebot_tasp.msg import mapdataPoint
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from setting import *

class GoAndTurn():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel',Twist,queue_size=10)

        self.position = Point()
        self.quat = Quaternion()
        self.move_cmd = Twist()

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(1.0))

        self.r1 = rospy.Rate(20)
        self.r2 = rospy.Rate(8)

        self.turn_to_goal(0)

    def turn_to_goal(self,goal_angle):
        (self.position, self.quat, present_angle) = self.get_odom()
        print('goal angle - present angle: '+str(goal_angle)+'  '+str(present_angle))
        while abs(goal_angle - present_angle) > 0.002:
            if goal_angle*present_angle < 0 and goal_angle < 0:
                if abs(goal_angle - present_angle) <= pi:
                    self.move_cmd.linear.x = 0
                    self.move_cmd.angular.z = -(OMEGA_1 + KA2*abs(goal_angle - present_angle))
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = OMEGA_1 + KA2*abs(goal_angle + present_angle)
            elif goal_angle*present_angle < 0 and goal_angle > 0:
                if abs(goal_angle -present_angle) <= pi:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = OMEGA_1 + KA2*abs(goal_angle - present_angle)
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -(OMEGA_1 + KA2*abs(goal_angle + present_angle))
            else:
                if goal_angle > present_angle:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = OMEGA_1 + KA2*abs(goal_angle - present_angle)
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -(OMEGA_1 + KA2*abs(goal_angle - present_angle))
            self.cmd_vel.publish(self.move_cmd)
            self.r1.sleep()
            (self.position, self.quat,present_angle) = self.get_odom()
        print('done')

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