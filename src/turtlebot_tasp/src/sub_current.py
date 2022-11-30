#!/usr/bin/env python3
import rospy
from os import path
from math import pow
import pandas as pd
from turtlebot3_msgs.msg import SensorState

current = 0
count = 0
analog = 0
data_list = []
cols_name = ['time','current']

def callback(data):
    global count, current, analog, data_list, cols_name
    if count < 9:
        analog += data.sonar
        count +=1
    else:
        count = 0
        analog = analog/9
        current = (( 5 / 1560 ) * analog - 2.5)/0.185
        rospy.loginfo(current)
        now = rospy.get_rostime()
        time = now.secs + now.nsecs*pow(10,-9)

        data = {
                'time': time,
                'current': current
            }
        data_list.append(data)

        analog = 0

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('sensor_state', SensorState, callback)
    rospy.spin()

if __name__ == '__main__':
    filename = input("Input the name of file for saving: ")
    listener()
    df = pd.DataFrame(data_list, columns=cols_name)
    csv_folder=path.dirname(__file__)
    df.to_csv(path.join(csv_folder,filename), index=False)
    print(' Done')