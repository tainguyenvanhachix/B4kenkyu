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
cols_name = ['time[s]', 'duration[s]', 'current[A]', 'voltage[V]', 'energy[J]', 'sum_energy[J]']

def callback(data):
    global count, current, analog, data_list, cols_name, last_time, time, sum_energy
    if count < 9:
        analog += data.sonar
        count +=1
    else:
        count = 0
        analog = analog/9
        current = (( 5 / 1560 ) * analog - 2.5)/0.185
        rospy.loginfo(current)
        now = rospy.get_rostime()
        if time == 0:
            time = now.secs + now.nsecs*pow(10,-9)
        last_time = time
        time = now.secs + now.nsecs*pow(10,-9)
        voltage = data.battery
        energy = current*voltage*(time - last_time)
        sum_energy = sum_energy+energy

        data = {
                'time[s]': time,
                'duration[s]': time - last_time,
                'current[A]': current,
                'voltage[V]': voltage,
                'energy[J]': energy,
                'sum_energy[J]': sum_energy
            }
        data_list.append(data)

        analog = 0

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('sensor_state', SensorState, callback)
    rospy.spin()

if __name__ == '__main__':
    filename = input("Input the name of file for saving: ")
    time = 0
    sum_energy = 0
    listener()
    df = pd.DataFrame(data_list, columns=cols_name)
    csv_folder=path.dirname(__file__)
    df.to_csv(path.join(csv_folder,filename), index=False)
    print(' Done')