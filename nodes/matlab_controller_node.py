#!/usr/bin/env python
import roslib

roslib.load_manifest('slytherin_dagger')

import rospy

import numpy as np
import random as rnd
import subprocess
import shlex

import sys
import os.path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import src.linear_predictor as predictor
import src.visual_features as feature
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class Controller:
    # ======== code for initializing the Controller ========

    # ----------------------------------------------------------------------
    # constructor
    # ----------------------------------------------------------------------
    def __init__(self):
        rospy.loginfo("[DAgger] Initializing controller")
        #load parameters from parameter server
        self.load_params()
        #init member variables to store sensor data
        self.init_variables()
        #load predictor
        if self.load_pred:
            rospy.loginfo("[DAgger] Loading Predictor")
            self.pred_yaw = predictor.load(self.predy_file)
            self.pred_pitch = predictor.load(self.predp_file)
            rospy.loginfo("[DAgger] Predictor Loaded")
        #init publisher for sending velocity commands
        self.init_publishers()
        #subscribe to topics for reading sensor data
        self.init_subscribers()
        #init timer for updating controls
        rospy.loginfo("[DAgger] Starting Control Loop")
        #rospy.Timer(rospy.Duration(1.0 / self.ctrl_rate_hz), self.update_control)


    #----------------------------------------------------------------------
    #desconstructor
    #----------------------------------------------------------------------
    def __del__(self):
        #stop recording bag file if we were
        if self.record_proc_started:
            self.record_proc.send_signal(subprocess.signal.SIGINT)
            self.record_proc.wait()

    #----------------------------------------------------------------------
    #init member variables
    #----------------------------------------------------------------------
    def init_variables(self):
        rospy.logdebug("[DAgger] Initializing Variables")
        self.is_auto = False
        self.record_proc_started = False
        self.last_vis_feat = np.array([])
        self.last_joy_vel = Twist()
        self.last_yaw = 0.0
        self.last_pitch = 0.0
        #dt = 1.0 / float(self.ctrl_rate_hz)

    #----------------------------------------------------------------------
    #load parameter from parameter server
    #----------------------------------------------------------------------
    def load_params(self):
        rospy.logdebug("[DAgger] Loading Params")
        self.load_pred = rospy.get_param('load_pred', default= False)
        self.predy_file = rospy.get_param('predy_file')
        self.predp_file = rospy.get_param('predp_file')
        self.ctrl_rate_hz = rospy.get_param('ctrl_rate_hz')
        self.expert_prob = rospy.get_param('expert_prob')
        self.pitch_gain = rospy.get_param('ang_y_gain')
        self.yaw_gain = rospy.get_param('ang_x_gain')
        self.do_record = rospy.get_param('do_record')
        self.record_dir = rospy.get_param('record_dir')
        self.pub_cmd_vel = rospy.get_param('pub_cmd_vel', default='sim_cmd_vel')
        self.pub_record = rospy.get_param('pub_record')
        self.pub_joy_vel = rospy.get_param('pub_joy_vel')
        self.pub_joy_start = rospy.get_param('pub_joy_start')
        self.pub_joy_stop = rospy.get_param('pub_joy_stop')
        self.pub_vis_feat = rospy.get_param('pub_vis_feat')

    #----------------------------------------------------------------------
    #subscribe callbacks to sensor data topics
    #----------------------------------------------------------------------
    def init_subscribers(self):
        rospy.logdebug("[DAgger] Initializing Subscribers")
        rospy.Subscriber(self.pub_joy_vel, Twist, self.joy_vel_update)
        rospy.Subscriber(self.pub_joy_start, Empty, self.joy_start_update)
        rospy.Subscriber(self.pub_joy_stop, Empty, self.joy_stop_update)
        rospy.Subscriber(self.pub_vis_feat, Float32MultiArray, self.vis_feat_update)

    #----------------------------------------------------------------------
    #initialize publisher to send velocity commands to quadrotor
    #----------------------------------------------------------------------
    def init_publishers(self):
        rospy.logdebug("[DAgger] Initializing Publishers")
        self.cmd_vel_publisher = rospy.Publisher(self.pub_cmd_vel, Twist)
        self.record_publisher = rospy.Publisher(self.pub_record, Float32MultiArray)


    #======== code for sensor data subscriber callback ========

    #----------------------------------------------------------------------
    #callback for visual feature update
    #----------------------------------------------------------------------
    def vis_feat_update(self, features):
        rospy.logdebug("[DAgger] Received Visual Feature Update: %s", np.array(features.data, dtype=np.float32))
        #create numpy array with visual features
        self.last_vis_feat = np.array(features.data, dtype=np.float32)

    #----------------------------------------------------------------------
    #callback for joystick velocity update
    #----------------------------------------------------------------------
    def joy_vel_update(self, vel):
        rospy.logdebug("[DAgger] Received Joystick Velocity Update: %s", vel)
        #store last velocity command from expert
        self.last_joy_vel = vel
        self.update_control()

    #----------------------------------------------------------------------
    #callback for start update
    #----------------------------------------------------------------------
    def joy_start_update(self, empty):  #start issued from joystick
        rospy.loginfo("[DAgger] Start Detected")
        self.is_auto = True
        self.last_yaw = 0.0
        self.last_pitch = 0.0

        if self.do_record:
            self.record_proc = subprocess.Popen(shlex.split("rosbag record -a"), cwd=self.record_dir)
            self.record_proc_started = True

    #----------------------------------------------------------------------
    #callback for stop update
    #----------------------------------------------------------------------
    def joy_stop_update(self, empty):  #stop issued from joystick
        rospy.loginfo("[DAgger] Stop Detected")
        self.is_auto = False
        if self.record_proc_started:
            self.record_proc.send_signal(subprocess.signal.SIGINT)

    #======== code for computing and sending controls to snake ========

    #----------------------------------------------------------------------
    #construct control msg to send to snake from output prediction
    #----------------------------------------------------------------------
    def construct_control_msg(self, pred_yaw, pred_pitch):
        ctrl = Twist()
        ctrl.linear.x = pred_yaw
        ctrl.linear.y = pred_pitch
        ctrl.linear.z = 0.0
        ctrl.angular.x = 0.0
        ctrl.angular.y = 0.0
        ctrl.angular.z = 0.0
        return ctrl

    #----------------------------------------------------------------------
    #computes new control command when in autonomous mode
    #----------------------------------------------------------------------
    def update_control_auto(self):
        #dt = 0
        #if not (event.last_real is None):
        #    dt = event.current_real.to_time() - event.last_real.to_time()

################################### construct feature array

        feat_array = self.last_vis_feat ### ..............find features..........
##########################################################
        expert_yaw = self.last_joy_vel.linear.x * self.yaw_gain
        expert_pitch = self.last_joy_vel.linear.y * self.pitch_gain
        pred_yaw = expert_yaw
        pred_pitch = expert_pitch
        #randomly pick whether we currently execute expert control or predictor control
        if self.load_pred and rnd.random() >= self.expert_prob:
            pred_yaw = self.pred_yaw.predict(feat_array)
            pred_pitch = self.pred_pitch.predict(feat_array)

        rospy.loginfo("[DAgger] predicted yaw: %f", pred_yaw)
        rospy.loginfo("[DAgger] expert yaw: %f", expert_yaw)
        rospy.loginfo("[DAgger] predicted yaw: %f", pred_pitch)
        rospy.loginfo("[DAgger] expert yaw: %f", expert_pitch)


        #record current datapoint for learning
        self.record(feat_array, expert_yaw, expert_pitch) ############## RECORDING FEATURES AND ACTION
        #send control message
        ctrl_msg = self.construct_control_msg(pred_yaw, pred_pitch)
        self.send_control_msg(ctrl_msg)
        self.last_yaw = pred_yaw
        self.last_pitch = pred_pitch

    #----------------------------------------------------------------------
    #callback function that compute and send control to snake using
    #latest sensor data
    #----------------------------------------------------------------------
    def update_control(self):
        if self.is_auto:
            rospy.loginfo("[DAgger] Auto running")
            self.update_control_auto()
        else:
            #pilot in control
            rospy.loginfo("[DAgger] Pilot in control")
            self.send_control_msg(self.last_joy_vel)

    #----------------------------------------------------------------------
    #send control to snake
    #----------------------------------------------------------------------
    def send_control_msg(self, ctrl_msg):
        rospy.logdebug("[DAgger] Sending control: %s", ctrl_msg)
        self.cmd_vel_publisher.publish(ctrl_msg)

    #======== code for recording data for future training ========

    #----------------------------------------------------------------------
    #record current feature vector with target yaw and pitch in record topic
    #----------------------------------------------------------------------
    def record(self, feat_array, pred_yaw, pred_pitch):
        if self.do_record:
            ar = np.append(feat_array, pred_yaw, pred_pitch)
            self.record_publisher.publish(None, ar)


if __name__ == '__main__':
    rospy.init_node('matlab_controller_node', log_level=rospy.DEBUG)
    ctrler = Controller()
    rospy.spin()


