#!/usr/bin/env python
''' Author: Tadele Belay MSc., Prof. Dr.-Ing Martin Manns
    (c) 2018 Universitity of Siegen, FAMS '''

import time
import datetime
import math
import numpy
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
'''
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
'''

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
       ==================
       
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

def direct_kinematics( th):
	
	A1=dh_matrix( 1,th)
	A2=dh_matrix( 2,th)
	A3=dh_matrix( 3,th)
	A4=dh_matrix( 4,th)
	A5=dh_matrix( 5,th)
	A6=dh_matrix( 6,th)

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
def interpolation(a, b): 
    return numpy.linspace(a, b, sampleSize, endpoint = True)







"""
''' Motion data '''
seek_point = [interpolation(J1_t, J1_f), interpolation(J2_t, J2_f),
              interpolation(J3_t, J3_f),interpolation(J4_t, J4_f),
              interpolation(J5_t, J5_f),interpolation(J6_t, J6_f)]
seek_point =numpy.array(seek_point)
  """
#seek_point_new = [seek_point[0, :],  seek_point[1, :], seek_point[2, :],
#              seek_point[3, :], seek_point[4, :], seek_point[5, :]]

        
'''PID Parameters'''
kp = 0.1
ki = 0
kd = 1

'''Set point force param'''
target_force = -100.0 

''' Client '''
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
    #global client
    try:
 
	ab = direct_kinematics(P1)	
	data = []
	time_delta = frange(0.0, 1.0, 0.01)
	for i in time_delta: 
		motion = lspb_motion(i, 0.0, 8.0, 0.0, 1.0)
		data.append(motion)
	data =  numpy.array(data)
	fig = plt.figure()
	plt.plot(data[:,1])
	plt.show()	
	print ab
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
    
    
if __name__ == '__main__': main()
