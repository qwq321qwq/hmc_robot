#!/usr/bin/env python
''' Author: Tadele Belay Tuli MSc.
    ======  E-mail: tadele-belay.tuli@uni-siegen.de
            Universitity of Siegen, Germany
            2019 
                    
    Note: 
    =====
          - This code has dependency on Net F/T sensor and UR5 ROS-system.'''

# Copyright Tadele Belay Tuli, 2019
# Distributed under the terms of the GNU General Public License

# --------------------------------------------------------------------
# This function is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This function is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with pyspread.  If not, see <http://www.gnu.org/licenses/>.
# -------------------------------------------------------------------
    

import time
import datetime
import numpy as np
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi
import matplotlib.pyplot as plt
from scipy import interpolate

""" ROS Package """

import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *


""" Modules """
from pid import *
from motion_planning import *
#Libraries for kinematics

""" Callback functions:
       ================
       
       Class 'Sensordata' = extract sensor data from robot internal sensors 
       and external force sensor

"""

class Sensordata:  
	
    def __init__(self):
        self.positions = 0.0
        self.force_sensor_z = 0.0
        self.torque_sensor_z  = 0.0
        self.force_tcp_r = 0.0
        self.torque_tcp_r =0.0
        
    def net_ft_sensor(self, data):
        """Force component from sensor data callback method        
        Stores force.z in class attribute
        """
        self.force_sensor_z = data.wrench.force.z
        self.torque_sensor_z = data.wrench.torque.z
    
    def robot_tcp_force(self, data):
        """ Force component from robot joint sensor data callback method       
        stores force and torque in class attribute"""
        self.force_tcp_r = data.wrench.force.z
        self.torque_tcp_r = data.wrench.torque.z

    def joint_positions(self, data):
        """Position component data callback method        
        Stores position as 6 tuple in class attribute
        """ 
        self.positions = data.position[0], data.position[1], data.position[2], data.position[3], data.position[4], data.position[5]


""" Data export functions:
    =====================
       
       Generates the txt file from the data
"""

def data_exporter(filename, data):
    """ This function is used to create text file for data export, by appending     
    each row in every iterations
    *\Input: 
    		filename = file name (string) e.g. filename = "force_data.txt"
    		data = data to be exported 
    """
    with open(filename, "a") as file_name:
         file_name.write("{}".format(data))
         file_name.write("\n")


""" 
Parameters of UR5:
================
       d = 
       a = 
       alpha = 
"""
global d, a, alpha
global mat
mat=np.matrix
d = mat([0.089159, 0, 0, 0.10915, 0.09465, 0.0823]) 
a =mat([0 ,-0.425 ,-0.39225 ,0 ,0 ,0]) 
alpha = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ])  

def dh_matrix( n, th): 
	"""
	Formulates the DH-matrix of n-joints
	"""
	Ta = mat(np.identity(4), copy=False)
	Ta[0,3] = a[0,n-1]
	Td = mat(np.identity(4), copy=False)
	Td[2,3] = d[0,n-1]
	
	Rz = mat([[cos(th[n-1]), -sin(th[n-1]), 0,  0],
	          [sin(th[n-1]), cos(th[n-1]), 0,  0],
	          [0,   0,   1,      0],
	          [0,   0,   0 ,     1]],copy=False)
	
	Rx = mat([[1,  0,                    0,                      0],
			     [0,  cos(alpha[0,n-1]),  -sin(alpha[0,n-1]),    0],
			     [0,  sin(alpha[0,n-1]),    cos(alpha[0,n-1]),   0],
			     [0,  0,        0,            1]],copy=False)
			     
	return Td * Rz * Ta * Rx

def direct_kinematics(th):
	
	A1=dh_matrix(1,th)
	A2=dh_matrix(2,th)
	A3=dh_matrix(3,th)
	A4=dh_matrix(4,th)
	A5=dh_matrix(5,th)
	A6=dh_matrix(6,th)

	T_06=A1*A2*A3*A4*A5*A6
	Pos = np.array(T_06[0:3, 3])
	return Pos.reshape(3)

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

"Position parameters "
HOME = [0, -1.57, 0, -1.57, 0, 0]	  	#Home Position
P1 = [52.02, -88.12, -107.78, -72.30, 89.26, -79.77] # Start
P2 = [57.02, -89.60, -111.30, -67.33, 89.26, -79.78] # Touch
P3 = [57.02, -89.58, -113.75, -60.82, 89.26, -79.78] # Stop


''' Interpolation function '''
sampleSize = 4000
def discretization(a, b): 
    
    return np.linspace(a, b, sampleSize, endpoint = True)


def move_robot(position, velocity, time_step):   
    """ This function takes a motion step argument and excutes the robot to the
    desired trajectory. 
    """
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES            
    g.trajectory.points = [
                JointTrajectoryPoint(positions=position, velocities=[velocity]*6,
                                     time_from_start=rospy.Duration(time_step))]    
    client.send_goal(g)               
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

	
'''Set point force param'''
target_force = -30.0 
client = None
class force_controller: 
    """ This class is used to implement PID force control based on the module  imported as PID 
    """        
    def pid_controller(self):
        """ This function accepts the state of the process and set_point and 
        throws the corresponding error
        """
        self.pid_out = PID(kp, ki, kd, 1, -1)

def frange(start, stop, step):
	""" This function yields the custom xrange for 
	floating numbers
	"""
	x = start
	while x < stop:
		yield x
		x += step
				    
def main():
    global client
    try:
        sensordata = Sensordata()
        
        '''
        Initialize ros node for the main file: "hmc_robot"
        '''
        rospy.init_node("hmc_robot", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', 
                                              FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        
        '''
        Launch the subscriber for:- 
            - robot tcp forc:            "/wrench"
            - external force sensor:     "/ft_sensor/netft_data"
            - robot joint state sensor:  "/joint_states"
        '''
        
        rospy.Subscriber("/wrench", WrenchStamped, sensordata.robot_tcp_force) 
        rospy.Subscriber("/ft_sensor/netft_data",  WrenchStamped, 
                         sensordata.net_ft_sensor)                              
        rospy.Subscriber("/joint_states", JointState, sensordata.joint_positions)
        
        '''
        Launch P-I-D controller 
        '''
        
        force_control = PID(0.1, 0.0, 1.0, 1.0,-1.0)
        t = datetime.datetime.now()
        
        '''
        Phase 1: Approaching to the object
            
            At this stage, the robot moves to the touch position
        '''
        pos_approach = [i * pi/180 for i in P2]           # Convert degree into rad
        
        joint_position = sensordata.positions             # Checks the joint state pos
        if joint_position != pos_approach: 
            move_robot(pos_approach, 0, 0.1)
            
        '''
        Phase 2: Target force seeking
        '''
        path_segments = [discretization(P2[i], P3[i]) for i in range(6)]
        
        ''' 
        Velocity is computed from trapezoidal velocity profile 
        '''               
        #time = velocity/path_segment 
        time = 0.03         # Let us check at real time speed of (33Hz)
        for i in range(sampleSize):
            # This loop implements high level position controller 
            sensor_force = sensordata.force_sensor_z 
            joint_force_tcp = sensordata.force_tcp_r 
            joint_pose = sensordata.positions
            th = mat(joint_pose)            
            error = sensor_force - target_force
            
            # Force PID Update
            force_control.update_PID(error)
                   
            if error == 0.0: 
                time.sleep(0.1)
            elif error > 0.0:
                ''' In this case, sensor force is less than target force and 
                the robot has to advance '''
                pos_seek = [path_segments[j][i] for j in range(6)]
                move_robot(pos_seek, 0, time)
                
                '''
                Phase 3: Force preservation
                '''
            else:
                ''' If there is a spikes on force e.g 29.0 -> 32.4, 
                    Use back-sliding position regulator '''
                    
                if sensor_force < target_force - 2.0:
                    if i > 2:
                        pos_seek = [path_segments[j][i-2] for j in range(6)]
                        move_robot(pos_seek, 0, time)
                elif sensor_force < target_force - 5.0:
                    if i > 5:
                        pos_seek = [path_segments[j][i-5] for j in range(6)]
                        move_robot(pos_seek, 0, time)
                else: 
                    time.sleep(2.0)
	    ''' 
            Implement data acquisition here!
            data_exporter(filename, data)
            '''
            time.sleep(15)
            print "The robto has achieved the maximum force"                    
            pos_initial = [i * pi/180 for i in P1] 
            move_robot(pos_initial)

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
    
    
if __name__ == '__main__': main()
