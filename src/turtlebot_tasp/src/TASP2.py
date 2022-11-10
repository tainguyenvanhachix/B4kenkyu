#!/usr/bin/env python3
import rospy
import math
from math import sqrt, pow, pi, atan2
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion

SAFE_TURN_DISTANCE = 0.530
CELL_LENGTH = 0.310
LINEAR_VEL = 0.08
ANGULAR_VEL = 0.2
SAMPLES_NUMBER = 1

class GoAndTurn():
    def __init__(self):
        rospy.init_node('TASP2', anonymous=False)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(30)
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(1.0))
        (goal_x, goal_y, goal_z) = (0,0,0)
        round = 0
        last_rotation = 0
        (position, rotation) = self.get_odom()
        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)
            last_distance = 5*CELL_LENGTH
            print('distance to wall: '+str(min_distance))
            if min_distance > SAFE_TURN_DISTANCE:
                print('chia: '+str(goal_z/(pi/2)%4))
                if (goal_z/(pi/2)) % 4 == 0:
                    goal_x += CELL_LENGTH
                    print('goalx, goaly, goalz: '+str(goal_x)+' '+str(goal_y)+' '+str(goal_z))
                elif (goal_z/(pi/2)) % 4 == 1:
                    goal_y += CELL_LENGTH
                    print('goalx, goaly, goalz: '+str(goal_x)+' '+str(goal_y)+' '+str(goal_z))
                elif (goal_z/(pi/2)) % 4 == 2:
                    goal_x -= CELL_LENGTH
                    print('goalx, goaly, goalz: '+str(goal_x)+' '+str(goal_y)+' '+str(goal_z))
                elif (goal_z/(pi/2)) % 4 == 3:
                    goal_y -= CELL_LENGTH
                    print('goalx, goaly, goalz: '+str(goal_x)+' '+str(goal_y)+' '+str(goal_z))
                distance = sqrt(pow((goal_x - position.x), 2) + pow((goal_y - position.y), 2))
                print('distance before: '+str(distance))
                while distance > 0.005:
                    (position, rotation2) = self.get_odom()
                    move_cmd.linear.x = LINEAR_VEL
                    move_cmd.angular.z = 0
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                    r.sleep()
                    distance = sqrt(pow((goal_x - position.x), 2) + pow((goal_y - position.y), 2))
                    if format(distance,".3f") > format(last_distance,".3f"):
                        print('distance: '+str(distance))
                        print('last_dis: '+str(last_distance))
                        print('break')
                        break
                    last_distance = distance
                print('distance later: '+str(distance))
                print('x, y, z after reach goal:'+str(position.x)+' '+str(position.y)+' '+str(rotation))
                print(' ')
            else:
                print('turn')
                goal_z += (pi/2)
                print('goal_z: '+str(goal_z))
                while abs(rotation - goal_z) > 0.01:
                    (position, rotation) = self.get_odom()
                    if abs((goal_z/(pi/2)) % 4) == 3:
                        if (rotation -last_rotation)>pi:
                            round -=1
                            print('round: '+str(round)+'   last_rotation: '+str(last_rotation)+ '   rotation: '+str(rotation/pi))
                        elif (rotation - last_rotation) < -pi:
                            round +=1
                            print('round: '+str(round)+'   last_rotation: '+str(last_rotation)+ '   rotation: '+str(rotation/pi))
                    last_rotation = rotation
                    rotation += round*2*pi
                    if rotation <= goal_z:
                        move_cmd.linear.x = 0.00
                        move_cmd.angular.z = ANGULAR_VEL
                    else:
                        move_cmd.linear.x = 0.00
                        move_cmd.angular.z = -ANGULAR_VEL
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                print('angular different later: '+str(goal_z-rotation ))
                print('x, y, z after reach goal:'+str(position.x)+' '+str(position.y)+' '+str(rotation))
                print(' ')
                move_cmd.linear.x = 0
                move_cmd.angular.z = 0
                self.cmd_vel.publish(move_cmd)

        rospy.loginfo("Stopping the robot...")
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        self.cmd_vel.publish(move_cmd)

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        while scan.intensities[0]==0:
            scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
        samples = len(scan.ranges)
        samples_view = SAMPLES_NUMBER
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

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

if __name__ == '__main__':
    try:
        print('start')
        GoAndTurn()

    except:
        rospy.loginfo("shutdown program.")