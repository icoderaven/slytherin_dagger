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

sys.path.append(roslib.packages.get_pkg_dir('slytherin_dagger')+'/src')
import linear_predictor as predictor
import visual_features as feature
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
        rospy.Timer(rospy.Duration(1.0 / self.ctrl_rate_hz), self.update_control)


    #----------------------------------------------------------------------
    #init member variables
    #----------------------------------------------------------------------
    def init_variables(self):
        rospy.logdebug("[DAgger] Initializing Variables")
        self.is_auto = False
        self.last_vis_feat = np.array([])
        self.last_joy_vel = Twist()
        self.last_yaw = 0.0
        self.last_pitch = 0.0
        dt = 1.0 / float(self.ctrl_rate_hz)

    #----------------------------------------------------------------------
    #load parameter from parameter server
    #----------------------------------------------------------------------
    def load_params(self):
        rospy.logdebug("[DAgger] Loading Params")
        self.load_pred = rospy.get_param('~load_pred')
        self.predy_file = rospy.get_param('~predy_file')
        self.predp_file = rospy.get_param('~predp_file')
        self.ctrl_rate_hz = rospy.get_param('~ctrl_rate_hz')
        self.expert_prob = rospy.get_param('~expert_prob')
        self.pitch_gain = rospy.get_param('~pitch_gain')
        self.pitch_max = rospy.get_param('~pitch_max')
        self.yaw_gain = rospy.get_param('~yaw_gain')
        self.yaw_max = rospy.get_param('~yaw_max')
        self.pub_cmd_vel = rospy.get_param('~pub_cmd_vel')
        self.pub_record = rospy.get_param('~pub_record')
        self.pub_joy_vel = rospy.get_param('~pub_joy_vel')
        self.pub_vis_feat = rospy.get_param('~pub_vis_feat')

    #----------------------------------------------------------------------
    #subscribe callbacks to sensor data topics
    #----------------------------------------------------------------------
    def init_subscribers(self):
        rospy.logdebug("[DAgger] Initializing Subscribers")
        rospy.Subscriber(self.pub_joy_vel, Twist, self.joy_vel_update)
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
    def update_control_auto(self, event):
        dt = 0
        if not (event.last_real is None):
            dt = event.current_real.to_time() - event.last_real.to_time()

################################### construct feature array
        feat_array = feature.find_2Dcenter(self.last_vis_feat)
##########################################################
        expert_yaw = self.last_joy_vel.angular.y * self.joy_yaw_gain
        expert_pitch = self.last_joy_vel.anglular.z * self.joy_pitch_gain
        pred__yaw = expert_yaw
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
        self.record(feat_array, expert_yaw, expert_pitch, pred_yaw, pred_pitch)
        #send control message
        ctrl_msg = self.construct_control_msg(pred_yaw, pred_pitch)
        self.send_control_msg(ctrl_msg)
        self.last_yaw = pred_yaw
        self.last_pitch = pred_pitch

    #----------------------------------------------------------------------
    #callback function that compute and send control to snake using
    #latest sensor data
    #----------------------------------------------------------------------
    def update_control(self, event):
        if self.is_auto:
            rospy.loginfo("[DAgger] Auto running")
            self.update_control_auto(event)
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
    def record(self, feat_array, expert_yaw, expert_pitch, pred_yaw, pred_pitch):
        if self.do_record:
            ar = np.append(feat_array, expert_yaw, expert_pitch, pred_yaw, pred_pitch)
            self.record_publisher.publish(None, ar)


if __name__ == '__main__':
    rospy.init_node('snake_controller_node', log_level=rospy.DEBUG)
    ctrler = Controller()
    rospy.spin()


