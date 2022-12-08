Install navigation package to have map_server package
- git clone https://github.com/ros-planning/navigation.git (branch: noetic-devel)
- catkin_make error
- sudo apt-get install libsdl-image1.2-dev
  sudo apt-get install libsdl-dev
- sudo apt-get install ros-noetic-tf2-sensor-msgs
- git clone https://github.com/ros-planning/navigation_msgs.git (branch: ros1)

Run
roscore

ssh ubuntu@172.20.10.12
roslaunch turtlebot3_bringup turtlebot3_robot.launch

roslaunch turtlebot3_bringup turtlebot3_remote.launch

roslaunch map_provider map_provider.launch 

rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 odom map

rosrun rviz rviz

rosrun map_provider occupancy_grid.py

rosrun turtlebot_tasp init.py

rosrun turtlebot_tasp sub_current.py

rosrun turtlebot_tasp TASP8.py 

rosrun turtlebot_tasp stop.py