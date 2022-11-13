#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

roll = pitch = yaw = 0.0

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)

r = rospy.Rate(10)
while not rospy.is_shutdown():    
    quat = quaternion_from_euler (roll, pitch,yaw)
    print('quat.z, quat.w, yaw: '+str(format(quat[2],".4f"))+'   '+str(format(quat[3],".4f"))+'   '+str(format(yaw,".4f")))
    r.sleep()