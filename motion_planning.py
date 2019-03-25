#!/usr/bin/env python
''' Author: Tadele Belay Tuli,  MSc., MSc.
    (c) 2018/9 University of Siegen, FAMS '''
   
import time
import datetime
import math
import numpy
import matplotlib.pyplot as plt

def frange(start, stop, step):
	""" This function yields the custom xrange for 
	floating numbers
	"""
	x = start
	while x < stop:
		yield x
		x += step

def discretize(a, b, sample): 
	""" This function creates segment points between a and b 
	using defined sample size"""
	
    	return numpy.linspace(a, b, sample, endpoint = True)		

		
def trapezoidal_velocity(t,  th0, thf, v_max):
	""" This function returns the trapezoidal motion plan: position, velocity, and acceleration
	"""
	# Boundary conditions (Note that t4-t3 or t2-t1 have to be <= (f4-t1)/2
	# And it can be computed as v_max / a_max   (acceleration time) 
	t1 = 0
	t2 = 1.0/3.0
	t3 = 2.0/3.0
	t4 = 1
		
	if t<= t2:
		th = th0 + v_max*(t-t1)**2 /(2.0* (t2 - t1))				# Trajectory
		th_dot =  v_max*(t-t1)/(t2 - t1)							# Velocity
		th_ddot = v_max /(t2 - t1)								    # Acceleration
		
	elif t>=t2 and t < t3:
		th = th0 + v_max*(t-t1-(t3 - t2)/2.0)
		th_dot =v_max
		th_ddot = 0
	else:
		th = thf - v_max *(t4 - t)**2 /(2.0*(t4 - t3))				# Check instead of thf using th0
		#th = th0 + v_max*(2.0/3.0*t4-t1-(t3 - t2)/2.0) + v_max*(t4*t - t**2)/(t4 - t3)
		th_dot =   v_max*(t4-t)/(t4 - t3)
		th_ddot = - v_max /(t4-t3)
	return th,  th_dot, th_ddot


def trapezoidal_velocity_with_lspb(t, th0, th1, t0, t1):
	""" This code is written based on a book of 'Robot model and Control' (By Spong). 
	The function is used to perform linear segment parabolic blend
	
	*\Parameters: 
	===========
	t 	= time step (time delta)
	th0 = initial joint position
	th1 = final joint position 
	t0	= initial time scale 
	t1	= final time scale
	"""
	vel = (th1-th0)/ t1 * 1.5;    	
    
    # If there is no change in position, 
	if th0 == th1:
		 th = th0
		 th_d = 0.0
		 th_dd = 0.0
		 return th, th_d, th_dd
	 
	# Determine the bending time  
	t_bending = (th0 - th1 + vel*t1)/vel
	acc = vel/t_bending

	if t <= t_bending:
	    # initial blend
	    th = th0 + acc/2.0*t**2
	    th_d = acc*t
	    th_dd = acc
	    
	elif t <= (t1-t_bending):
	    # linear motion
	    th = (th1+th0-vel*t1)/2.0 + vel*t;
	    th_d = vel;
	    th_dd = 0;
	    
	else:
	    # final blend
	    th = th1 - acc/2.0*t1**2 + acc*t1*t - acc/2.0*t**2;
	    th_d = acc*t1 - acc*t;
	    th_dd  = -acc;
	    
	return th, th_d, th_dd




	
	
