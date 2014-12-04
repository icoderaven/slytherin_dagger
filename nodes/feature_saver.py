#!/usr/bin/env python
'''
Simple utility to convert images to features using feature generator node and recorded bag file
'''
import roslib

roslib.load_manifest('slytherin_dagger')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from cv_bridge import CvBridge, CvBridgeError
import cv2
import rosbag


class BagSaver:
    def __init__(self):
        self.joy_topic = rospy.get_param('~joy_topic', default='/cmd_vel')
        self.feat_topic = rospy.get_param('~feat_topic', default='/feature')
        self.bag_file_path = rospy.get_param('~bag_file_path',
                                             default='/media/Obelix/GraduateSchool/FallCourses14/16831/project/snake_robot_project/FeatureBagFiles/FeatureBagFiles' + str(rospy.Time.now().secs) + '.bag')

        self.last_joy_msg = None
        self.last_feat_msg = None
        self.last_time = rospy.Time.now()
        self.bag_file = rosbag.Bag(self.bag_file_path, 'w')
        self.joy_sub = rospy.Subscriber(self.joy_topic, Twist, self.joy_update, queue_size=1)
        self.feat_sub = rospy.Subscriber(self.feat_topic, Float32MultiArray, self.feat_update, queue_size=1)


    def __del__(self):
        self.bag_file.close()

    def close(self):
        self.bag_file.close()

    def joy_update(self, data):
        rospy.loginfo('[Feature_saver] received Joystick data')
        self.last_joy_msg = data
        self.bag_file.write(self.joy_topic, self.last_joy_msg)

    def feat_update(self, data):
        rospy.loginfo('[Feature_saver] received feature data')
        self.last_feat_msg = data
        self.bag_file.write(self.feat_topic, self.last_feat_msg)

if __name__ == '__main__':
    rospy.init_node('feature_saver')
    rospy.loginfo('started feature saver')
    my_saver = BagSaver()
    rospy.spin()
    my_saver.close()
