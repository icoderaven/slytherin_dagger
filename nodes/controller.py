#!/usr/bin/env python  
import roslib
roslib.load_manifest('ardrone_dagger_im')
import rospy

import math
import numpy as np
import random as rnd

import subprocess
import shlex
#this code can use different type of predictors
#the predictor .py file needs to implement 
# - a global method load, which takes a filename as input and return a ``predictor'' object as output
# - the ``predictor'' object should implement a predict method, which takes a numpy.array as input and outputs some number as output
#see linear_predictor.py for an example
#to abstract the type of predictor, simply import the predictor .py file as predictor
#e.g. import linear_predictor as predictor
#NOTE: maybe a better way to do this is to setup the predictor as a service
import linear_predictor as predictor 

#this code can also use different ways of constructing features
#the feature constructor .py file needs to implement 
# - a global method construct, which takes 4 numpy.array as input (visual features, position, quaternion and goal) 
#   and return a feature vector (numpy.array) as output
#see default_feature_constructor.py for an example
#to abstract the type of feature constructor, simply import the feature constructor .py file as feature
#e.g. import default_feature_constructor as feature
#NOTE: maybe a better way to do this is to setup the feature constructor as a service
import default_feature_constructor as feature 

import utils

from ardrone_pid.srv import *
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from navdata_feature import NavdataFeature

class Controller:
	#======== code for initializing the Controller ========

	#----------------------------------------------------------------------
	#constructor
	#----------------------------------------------------------------------
	def __init__(self):
		rospy.loginfo("[DAgger] Initializing controller")
		#load parameters from parameter server
		self.load_params()
		#init member variables to store sensor data
		self.init_variables()
		#load predictor
		if self.m_load_pred:
			rospy.loginfo("[DAgger] Loading Predictor")
			self.m_pred = predictor.load(self.m_pred_file)
			rospy.loginfo("[DAgger] Predictor Loaded")
		#init publisher for sending velocity commands
		self.init_publishers()
		#subscribe to topics for reading sensor data
		self.init_subscribers()
		#init timer for updating controls
		rospy.loginfo("[DAgger] Starting Control Loop")
		rospy.Timer(rospy.Duration(1.0 / self.m_ctrl_rate_hz), self.update_control)

	#----------------------------------------------------------------------
	#desconstructor
	#----------------------------------------------------------------------
	def __del__(self):
		#make sure the quadrotor lands before exiting DAgger		
		self.m_land_publisher.publish(Empty())
		#stop recording bag file if we were
		if self.m_record_proc_started:
			self.m_record_proc.send_signal(subprocess.signal.SIGINT) 
			self.m_record_proc.wait()

	#----------------------------------------------------------------------
	#init member variables
	#----------------------------------------------------------------------
	def init_variables(self):
		rospy.logdebug("[DAgger] Initializing Variables")
		self.m_is_flying = self.m_is_starting_flying
		self.m_is_auto = False
		self.m_record_proc_started = False
		self.m_goal[2] = self.m_flying_altitude
		self.m_last_vis_feat = np.array([])
		self.m_last_joy_vel = Twist()
		self.m_last_mocap_posestmp = PoseStamped()
		self.m_prev_last_mocap_posestmp = PoseStamped()
		self.m_position_pid = self.m_last_mocap_posestmp.pose.position
		dt = 1.0 / float(self.m_ctrl_rate_hz)
		self.m_filters_K = np.zeros(self.m_nb_ctrl_filters)
		self.m_filtered_prv_ctrl = np.zeros(self.m_nb_ctrl_filters)
		self.m_last_lin_y = 0.0
		for i in range(self.m_nb_ctrl_filters):
			self.m_filters_K[i] = (pow(self.m_ctrl_filters_pow, i) - 1.0) * dt
			


	#----------------------------------------------------------------------	
	#load parameter from parameter server
	#----------------------------------------------------------------------
	def load_params(self):
		rospy.logdebug("[DAgger] Loading Params")
		self.m_is_starting_flying = rospy.get_param('~is_starting_flying')
		self.m_in_mocap = rospy.get_param('~in_mocap')
		self.m_load_pred = rospy.get_param('~load_pred')
		self.m_pred_file = rospy.get_param('~pred_file')
		self.m_ctrl_rate_hz = rospy.get_param('~ctrl_rate_hz')
		self.m_expert_prob = rospy.get_param('~expert_prob')
		self.m_take_off_delay = rospy.get_param('~takeoff_delay')
		self.m_joy_lin_y_gain = rospy.get_param('~joy_lin_y_gain')
		self.m_ang_z_gain = rospy.get_param('~ang_z_gain')
		self.m_ang_z_max = rospy.get_param('~ang_z_max')
		self.m_thresh_ang_fwd = rospy.get_param('~thresh_ang_fwd')
		self.m_fwd_vel_gain = rospy.get_param('~fwd_vel_gain')
		self.m_fwd_vel_max = rospy.get_param('~fwd_vel_max')
		self.m_flying_altitude = rospy.get_param('~flying_altitude')
		self.m_altitude_gain = rospy.get_param('~altitude_gain')
		self.m_altitude_vel_max = rospy.get_param('~altitude_vel_max')
		self.m_nb_ctrl_filters = rospy.get_param('~nb_ctrl_filters')
		self.m_ctrl_filters_pow = rospy.get_param('~ctrl_filters_pow')
		self.m_goal = np.zeros(3)
		self.m_goal[0] = rospy.get_param('~default_goal_x')
		self.m_goal[1] = rospy.get_param('~default_goal_y')
		self.m_goal[2] = rospy.get_param('~default_goal_z')
		self.m_goal_metric = np.zeros(3)
		self.m_goal_metric[0] = rospy.get_param('~goal_metric_x')
		self.m_goal_metric[1] = rospy.get_param('~goal_metric_y')
		self.m_goal_metric[2] = rospy.get_param('~goal_metric_z')
		self.m_is_goal_broadcast = rospy.get_param('~is_goal_broadcast')
		self.m_goal_radius = rospy.get_param('~goal_radius')
		self.m_do_record = rospy.get_param('~do_record')
		self.m_record_dir = rospy.get_param('~record_dir')
		self.m_pub_cmd_vel = rospy.get_param('~pub_cmd_vel') #"/cmd_vel"
		self.m_pub_record = rospy.get_param('~pub_record') #"/dagger_record"
		self.m_pub_joy_vel = rospy.get_param('~pub_joy_vel') #"/joy_vel"
		self.m_pub_joy_takeoff = rospy.get_param('~pub_joy_takeoff') #"/ardrone/takeoff"
		self.m_pub_joy_land = rospy.get_param('~pub_joy_land') #"/ardrone/land"
		self.m_pub_joy_reset = rospy.get_param('~pub_joy_reset') #"/ardrone/reset"
		self.m_pub_joy_start = rospy.get_param('~pub_joy_start') #"/ardrone/start"
		self.m_pub_joy_stop = rospy.get_param('~pub_joy_stop') #"/ardrone/stop"
		self.m_pub_vis_feat = rospy.get_param('~pub_vis_feat') #"/birdvision/features"
		self.m_pub_mocap = rospy.get_param('~pub_mocap') #"/mocap_retransmit"
		self.m_pub_goal = rospy.get_param('~pub_goal') #"/goal_location"
		self.m_srv_pid = rospy.get_param('~srv_pid') #"pid_commands"
		

	#----------------------------------------------------------------------
	#subscribe callbacks to sensor data topics 
	#----------------------------------------------------------------------
	def init_subscribers(self):
		rospy.logdebug("[DAgger] Initializing Subscribers")
		rospy.Subscriber(self.m_pub_mocap, PoseStamped, self.mocap_update)
		rospy.Subscriber(self.m_pub_joy_vel, Twist, self.joy_vel_update)
		rospy.Subscriber(self.m_pub_joy_takeoff, Empty, self.joy_takeoff_update)
		rospy.Subscriber(self.m_pub_joy_land, Empty, self.joy_land_update)
		rospy.Subscriber(self.m_pub_joy_reset, Empty, self.joy_reset_update)
		rospy.Subscriber(self.m_pub_joy_start, Empty, self.joy_start_update)
		rospy.Subscriber(self.m_pub_joy_stop, Empty, self.joy_stop_update)
		rospy.Subscriber(self.m_pub_vis_feat, Float32MultiArray, self.vis_feat_update)
		if self.m_is_goal_broadcast:
			rospy.Subscriber(self.m_pub_goal, PoseStamped, self.goal_update)

	#----------------------------------------------------------------------
	#initialize publisher to send velocity commands to quadrotor 
	#----------------------------------------------------------------------
	def init_publishers(self):
		rospy.logdebug("[DAgger] Initializing Publishers")
		self.m_land_publisher = rospy.Publisher(self.m_pub_joy_land, Empty)
		self.m_cmd_vel_publisher = rospy.Publisher(self.m_pub_cmd_vel, Twist)
		self.m_record_publisher = rospy.Publisher(self.m_pub_record, Float32MultiArray)


	#======== code for client ======

	def pid_command_client(self, x, y, z, last_posestmp, cur_posestmp):
		rospy.wait_for_service(self.m_srv_pid)
		try:
			pid_commands = rospy.ServiceProxy(self.m_srv_pid, PidCommands)
			resp1 = pid_commands(x, y, z, last_posestmp, cur_posestmp)
			return resp1.uav_comm_vel
		except rospy.ServiceException, e:
			rospy.loginfo("[DAgger] pid_commands Service call failed: %s", e)


	#======== code for sensor data subscriber callback ========

	#----------------------------------------------------------------------
	#callback for visual feature update
	#----------------------------------------------------------------------	
	def vis_feat_update(self, features):
		rospy.logdebug("[DAgger] Received Visual Feature Update: %s", np.array(features.data, dtype=np.float32))
		#create numpy array with visual features
		self.m_last_vis_feat = np.array(features.data, dtype=np.float32)

	#----------------------------------------------------------------------		
	#callback for joystick velocity update
	#----------------------------------------------------------------------	
	def joy_vel_update(self, vel):
		rospy.logdebug("[DAgger] Received Joystick Velocity Update: %s", vel)
		#store last velocity command from expert
		self.m_last_joy_vel = vel

	#----------------------------------------------------------------------		
	#callback for takeoff update
	#----------------------------------------------------------------------	
	def joy_takeoff_update(self, empty): #takeoff issued from joystick		
		rospy.loginfo("[DAgger] Takeoff Detected")
		#set position to maintain by the pid during takeoff
		self.m_position_pid = self.m_last_mocap_posestmp.pose.position
		self.m_position_pid.z = self.m_flying_altitude
		#wait for takeoff to occur		
		#rospy.sleep(self.m_take_off_delay)
		self.m_is_flying = True

	#----------------------------------------------------------------------		
	#callback for landing update
	#----------------------------------------------------------------------	
	def joy_land_update(self, empty): #land issued from joystick
		rospy.loginfo("[DAgger] Landing Detected")
		self.m_is_flying = False
		self.m_is_auto = False
		if self.m_record_proc_started:
			self.m_record_proc.send_signal(subprocess.signal.SIGINT)

	#----------------------------------------------------------------------		
	#callback for reset update
	#----------------------------------------------------------------------	
	def joy_reset_update(self, empty): #e-stop issued from joystick
		rospy.loginfo("[DAgger] Reset Detected")
		self.m_is_flying = False
		self.m_is_auto = False
		if self.m_record_proc_started:
			self.m_record_proc.send_signal(subprocess.signal.SIGINT)

	#----------------------------------------------------------------------		
	#callback for start update
	#----------------------------------------------------------------------	
	def joy_start_update(self, empty): #start issued from joystick
		rospy.loginfo("[DAgger] Start Detected")
		self.m_is_auto = True
		self.m_filtered_prv_ctrl = np.zeros(self.m_nb_ctrl_filters)
		self.m_last_lin_y = 0.0
		self.navdata_feature = NavdataFeature()
	 	#Set yaw direction to target on start
		
		if self.m_do_record:
			self.m_record_proc = subprocess.Popen(shlex.split("rosbag record -a"), cwd=self.m_record_dir)
			self.m_record_proc_started = True

	#----------------------------------------------------------------------		
	#callback for stop update
	#----------------------------------------------------------------------	
	def joy_stop_update(self, empty): #stop issued from joystick
		#set position to maintain by the pid after stopping auto mode
		self.m_position_pid = self.m_last_mocap_posestmp.pose.position
		self.m_position_pid.z = self.m_flying_altitude
		rospy.loginfo("[DAgger] Stop Detected")
		self.m_is_auto = False
		if self.m_record_proc_started:
			self.m_record_proc.send_signal(subprocess.signal.SIGINT)

	#----------------------------------------------------------------------		
	#callback for mocap position update
	#----------------------------------------------------------------------	
	def mocap_update(self, posestmp):
		rospy.logdebug("[DAgger] Received new Mocap Pose: %s", posestmp.pose)
		#update last mocap data
		self.m_prev_last_mocap_posestmp = self.m_last_mocap_posestmp
		self.m_last_mocap_posestmp = posestmp

	#----------------------------------------------------------------------		
	#callback for goal position update
	#----------------------------------------------------------------------	
	def goal_update(self, posestmp):
		rospy.logdebug("[DAgger] Received new Goal Pose: %s", posestmp.pose)
		#update last mocap data
		self.m_goal[0] = posestmp.pose.position.x
		self.m_goal[1] = posestmp.pose.position.y


	#======== code for computing and sending controls to quadrotor ========
		
	#----------------------------------------------------------------------		
	#construct control msg to send to quadrotor from output prediction
	#----------------------------------------------------------------------
	def construct_control_msg(self, fwd_vel, vel_z, ang_z, pred_lin_y):	
		ctrl = Twist()
		ctrl.linear.x = fwd_vel
		ctrl.linear.y = pred_lin_y
		ctrl.linear.z = vel_z
		ctrl.angular.x = 0
		ctrl.angular.y = 0
		ctrl.angular.z = ang_z
		return ctrl

	#----------------------------------------------------------------------
	#computes new control command when in autonomous mode in mocap arena	
	#----------------------------------------------------------------------
	def update_control_auto_mocap(self, event):
		pos = utils.convert_position_to_array(self.m_last_mocap_posestmp.pose.position)
		q = utils.convert_orientation_to_array(self.m_last_mocap_posestmp.pose.orientation)
		ang_goal = feature.compute_goal_dir(pos, q, self.m_goal)
		dt = 0
		if not (event.last_real is None):
			dt = event.current_real.to_time() - event.last_real.to_time()
		#update features history ctrl
		for i in range(self.m_nb_ctrl_filters):
			self.m_filtered_prv_ctrl[i] = feature.low_pass_filter(self.m_filtered_prv_ctrl[i], self.m_last_lin_y, dt, self.m_filters_K[i])
		#check whether we need to adjust altitude and heading
		zerr = pos[2] - self.m_flying_altitude
		vel_z = zerr * self.m_altitude_gain
		vel_z = min(vel_z, self.m_altitude_vel_max)
		vel_z = max(vel_z, -self.m_altitude_vel_max)
		ang_z = ang_goal * self.m_ang_z_gain
		ang_z = min(ang_z, self.m_ang_z_max)
		ang_z = max(ang_z, -self.m_ang_z_max)
		rospy.loginfo("[DAgger] Vel z: %f Ang z: %f", vel_z, ang_z)
		#get features for predicting angular vel
		feat_array = feature.construct(self.m_last_vis_feat, self.m_filtered_prv_ctrl)#, pos, q, self.m_goal)
		#randomly pick whether we currently execute expert control or predictor control
		expert_lin_y = self.m_last_joy_vel.linear.y * self.m_joy_lin_y_gain 
		pred_lin_y = expert_lin_y
		if self.m_load_pred and self.m_expert_prob < 1 and (self.m_expert_prob == 0 or rnd.random() > self.m_expert_prob):
			pred_lin_y = self.m_pred.predict(feat_array)
		rospy.loginfo("[DAgger] lin y: %f", pred_lin_y)
		#compute forward velocity				
		fwd_vel = 0
		if ang_goal > -self.m_thresh_ang_fwd and ang_goal < self.m_thresh_ang_fwd:
			fwd_vel = self.compute_distance_goal(self.m_last_mocap_posestmp.pose, self.m_goal, self.m_goal_metric) * self.m_fwd_vel_gain
			fwd_vel = min(fwd_vel, self.m_fwd_vel_max)
			fwd_vel = max(fwd_vel, 0)
		rospy.loginfo("[DAgger] fwd vel: %f", fwd_vel)
		#record current datapoint if not at goal
		if self.is_at_goal(self.m_last_mocap_posestmp.pose, self.m_goal, self.m_goal_metric):
			rospy.loginfo("[DAgger] Reached Goal Location")
			fwd_vel = 0
			ang_z = 0
			vel_z = 0
			pred_lin_y = 0
			self.m_land_publisher.publish(Empty())
		else:
			self.record(feat_array, expert_lin_y)
			ctrl_msg = self.construct_control_msg(fwd_vel, vel_z, ang_z, pred_lin_y)
			self.send_control_msg(ctrl_msg)
		self.m_last_lin_y = pred_lin_y

	#----------------------------------------------------------------------
	#computes new control command when in autonomous mode but not in mocap arena	
	#----------------------------------------------------------------------
	def update_control_auto_openloop(self, event):
		dt = 0
		if not (event.last_real is None):
			dt = event.current_real.to_time() - event.last_real.to_time()
		#update features history ctrl
		for i in range(self.m_nb_ctrl_filters):
			self.m_filtered_prv_ctrl[i] = feature.low_pass_filter(self.m_filtered_prv_ctrl[i], self.m_last_lin_y, dt, self.m_filters_K[i])
		# get features for predicting angular vel
		#feat_array = feature.construct(self.m_last_vis_feat, self.m_filtered_prv_ctrl)
		
		# feat_array = [visual features, yaw offset from requested value,
		#               current y velocity, vector of previous commands, offset = 1] 
		if self.m_last_vis_feat.size == 0:
			return
		(feat_array, feat_weights) = feature.construct_weighted(self.m_last_vis_feat, self.m_filtered_prv_ctrl, self.navdata_feature.yaw_error, self.navdata_feature.y_velocity)
		
		#randomly pick whether we currently execute expert control or predictor control
		expert_lin_y = self.m_last_joy_vel.linear.y * self.m_joy_lin_y_gain 
		pred_lin_y = expert_lin_y
		if self.m_load_pred and self.m_expert_prob < 1 and (self.m_expert_prob == 0 or rnd.random() > self.m_expert_prob):
			pred_lin_y = self.m_pred.predict(feat_array)
		rospy.loginfo("[DAgger] lin y: %f", pred_lin_y)
		rospy.loginfo("[DAgger] exp lin y: %f", expert_lin_y)
		rospy.loginfo("[DAgger] Directional error: %f", self.navdata_feature.yaw_error)
		#in open loop we simply go forward at fixed velocity and allow pilot to adjust altitude
		fwd_vel = self.m_fwd_vel_max
		vel_z = self.m_last_joy_vel.linear.z
		rospy.loginfo("[DAgger] fwd vel: %f", fwd_vel)
		#record current datapoint
		self.record(feat_array, expert_lin_y)
		#send control message		
		ang_z = 1.0 * self.navdata_feature.yaw_error
		ctrl_msg = self.construct_control_msg(fwd_vel, vel_z, ang_z, pred_lin_y)
		self.send_control_msg(ctrl_msg)
		self.m_last_lin_y = pred_lin_y


	#----------------------------------------------------------------------
	#callback function that compute and send control to quadrotor using 
	#latest sensor data	
	#----------------------------------------------------------------------
	def update_control(self, event):
		if self.m_is_flying:
			if self.m_is_auto:
				rospy.logdebug("[DAgger] Computing new angular velocity")
				rospy.loginfo("[DAgger] Auto running")
				if self.m_in_mocap:
					self.update_control_auto_mocap(event)
				else:
					self.update_control_auto_openloop(event)
			else: 
				if self.m_in_mocap:
					#pid is in control and seeks to maintain m_position_pid
					cmd = self.pid_command_client(self.m_position_pid.x, self.m_position_pid.y, self.m_position_pid.z, self.m_prev_last_mocap_posestmp, self.m_last_mocap_posestmp)
					self.send_control_msg(cmd)
				else:
					#pilot in control
					rospy.loginfo("[DAgger] Pilot in control")
					self.send_control_msg(self.m_last_joy_vel)

	#----------------------------------------------------------------------
	#send control to quadrotor 
	#----------------------------------------------------------------------
	def send_control_msg(self, ctrl_msg):
		rospy.logdebug("[DAgger] Sending control: %s", ctrl_msg)
		self.m_cmd_vel_publisher.publish(ctrl_msg)

	#======== code for recording data for future training ========

	#----------------------------------------------------------------------
	#record current feature vector with target linear velocity in record topic
	#----------------------------------------------------------------------	
	def record(self, feat_array, lin_y):
		if self.m_do_record:
			ar = np.append(feat_array, lin_y)
			self.m_record_publisher.publish(None, ar)

	#======== utility functions =======

	#----------------------------------------------------------------------
	#compute distance between current pose of quadrotor and goal location
	#----------------------------------------------------------------------	
	def compute_distance_goal(self, pose, goal, metric):
		pos = utils.convert_position_to_array(pose.position)
		return np.linalg.norm((pos - goal) * np.sqrt(metric))

	#----------------------------------------------------------------------
	#return True if at goal
	#----------------------------------------------------------------------	
	def is_at_goal(self, pose, goal, metric):
		d = self.compute_distance_goal(pose, goal, metric)
		return d <= self.m_goal_radius



if __name__ == '__main__':
	rospy.init_node('controller', log_level=rospy.DEBUG)
	ctrler = Controller()
	rospy.spin()	


