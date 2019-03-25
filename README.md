# Hierarchical Motion Control for Industrial Robots Using ROS
Hierarchical motion control for real time simulation of industrial robots

In order to use this code, please follow the following,
# Clone universal robot ROS package 
For the ROS package of Universal Robot, check at https://github.com/ros-industrial/universal_robot.git'

# Clone Net F/T Sensor ROS package 
For the ROS package of Net F/T sensor check at: https://github.com/epfl-lasa/net-ft-ros.git

# How to use the code: 
- Launch ROS using roscore

- Launch Universal robot and configure the IP-Address from ROS 

- Launch Net F/T Sensor and configure the IP - Address from ROS

- Check if the connection is established. From terminal use: ping 192.168.1.10 / 192.168.1.1 based on the IP Address of the configuration

- If the connection is successful - before you continue check for simple test to check the system.

- Then RUN main_file.py and visualize your data based on your requirment. 

# Result

- Experimental setup and system configuration:

![Experimental_setup](IMG_1351.jpg)


- Real time simulation 
