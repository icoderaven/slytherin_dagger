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
import pdb
sys.path.append(roslib.packages.get_pkg_dir('slytherin_dagger')+'/src')
from linear_predictor import LinearPredictor
import visual_features as feature
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2

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
            self.pred_yaw = LinearPredictor()
            self.pred_yaw.load(self.predy_file)
            self.pred_pitch = LinearPredictor()
            self.pred_pitch.load(self.predp_file)
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
        self.is_auto = True
        self.last_vis_feat = None
        self.last_joy_vel = None
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
        self.bridge = CvBridge()
        rospy.Subscriber(self.pub_joy_vel, Twist, self.joy_vel_update)
        rospy.Subscriber(self.pub_vis_feat, Image, self.vis_feat_update)

    #----------------------------------------------------------------------
    #initialize publisher to send velocity commands to quadrotor
    #----------------------------------------------------------------------
    def init_publishers(self):
        rospy.logdebug("[DAgger] Initializing Publishers")
        self.cmd_vel_publisher = rospy.Publisher(self.pub_cmd_vel, Twist, queue_size=1)
        self.record_publisher = rospy.Publisher(self.pub_record, Float32MultiArray, queue_size=1)


    #======== code for sensor data subscriber callback ========

    #----------------------------------------------------------------------
    #callback for visual feature update
    #----------------------------------------------------------------------
    def vis_feat_update(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
        #create numpy array with visual features
        self.last_vis_feat = cv_image

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
        
        if (self.last_vis_feat is None) or (self.last_joy_vel is None):
            rospy.loginfo('[DAgger] Haven\'t received image or joy data')
            return 
################################### construct feature array
        feat_array = feature.findholecentre(self.last_vis_feat)
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
            rospy.loginfo("[DAgger] predicted pitch: %f", pred_pitch)
            rospy.loginfo("[DAgger] expert pitch: %f\n", expert_pitch)

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
            #rospy.loginfo("[DAgger] Auto running")
            self.update_control_auto(event)
        else:
            #pilot in control
            #rospy.loginfo("[DAgger] Pilot in control")
            if self.last_joy_vel is not None:
                self.send_control_msg(self.last_joy_vel)
                self.last_joy_vel = None

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
        ar = np.append(feat_array, (expert_yaw, expert_pitch, pred_yaw, pred_pitch))
        self.record_publisher.publish(None, ar)


if __name__ == '__main__':
    rospy.init_node('snake_controller_node')#, log_level=rospy.DEBUG)
    ctrler = Controller()
    rospy.spin()


