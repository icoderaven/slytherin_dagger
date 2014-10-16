#!/usr/bin/env python  
import math
import numpy as np

#----------------------------------------------------------------------
#converts angles in degrees to radians
#----------------------------------------------------------------------	
def deg_to_rad(angle):
	return angle*math.pi/180.0

#----------------------------------------------------------------------
#converts angles in radians to degrees
#----------------------------------------------------------------------	
def rad_to_deg(angle):
	return angle*180.0/math.pi

#----------------------------------------------------------------------
#converts ROS Point/Vector3 object to a numpy array
#----------------------------------------------------------------------	
def convert_position_to_array(position):
	pos = np.zeros(3)
	pos[0] = position.x
	pos[1] = position.y
	pos[2] = position.z
	return pos

#----------------------------------------------------------------------
#converts ROS Quaternion object to a numpy array
#----------------------------------------------------------------------	
def convert_orientation_to_array(orientation):
	q = np.zeros(4)
	q[0] = orientation.x
	q[1] = orientation.y
	q[2] = orientation.z
	q[3] = orientation.w
	return q
