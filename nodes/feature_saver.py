#!/usr/bin/env python
'''
Simple utility to convert images to features using feature generator node and recorded bag file
'''
import roslib

roslib.load_manifest('slytherin_dagger')
import rospy
import sys
sys.path.append(roslib.packages.get_pkg_dir('slytherin_dagger')+'/src')
import feature_generator as feature
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from cv_bridge import CvBridge, CvBridgeError
import cv2
import rosbag
import os
import numpy as np

if __name__ == '__main__':
    rospy.init_node('feature_saver')
    rospy.loginfo('started feature saver')
    list_of_files = rospy.get_param('~list_of_files')
    path_bag = rospy.get_param('~bag_folder')
    camera_topic = rospy.get_param('~camera_topic', default='/camera/image_raw')
    joy_topic    = rospy.get_param('~joy_topic', default='/cmd_vel')
    record_topic    = rospy.get_param('~record_topic', default='/record')
    bridge = CvBridge()
    f = open(list_of_files, 'r')
    # load all bags in f
    counter = 0
    for line in f:
        # open the current bag file
        line2 = line.rstrip(' \t\n')
        rospy.loginfo("[DAgger] Opening bag file %s", path_bag + line2)
        try:
            bag = rosbag.Bag(path_bag + line2)
        except rosbag.bag.ROSBagUnindexedException:
            rospy.loginfo("[DAgger] Unindexed Bag file %s. Attempting to reindex", path_bag + line2)
            call(shlex.split("rosbag reindex %s" % (path_bag + line2)))
            try:
                bag = rosbag.Bag(path_bag + line2)
                rospy.loginfo("[DAgger] Reindexing Succesful")
            except rosbag.bag.ROSBagUnindexedException:
                rospy.loginfo("[DAgger] Reindexing failed, skipping file %s", path_bag + line2)
                continue
        name, ext = os.path.splitext(line2)
        write_bag = rosbag.Bag(path_bag + name + '_featured' + ext, 'w')
        # look at msg in dagger_record topic
        camera_msg = None
        joy_msg = None
        for topic, msg, t in bag.read_messages(topics=[camera_topic, joy_topic]):
            if topic == camera_topic:
                camera_msg = msg
            if topic == joy_topic:
                joy_msg = msg

            if camera_msg is not None and joy_msg is not None:
                #generate features  
                #convert msg.data to a numpy array
                ar = np.array(bridge.imgmsg_to_cv2(camera_msg,desired_encoding='passthrough'), dtype=np.uint8)
                bw_img = cv2.cvtColor(ar, cv2.COLOR_RGB2GRAY)
                feature_msg = Float32MultiArray()
                yaw = np.array(joy_msg.linear.x, dtype=np.float32)
                pitch = np.array(joy_msg.linear.y, dtype=np.float32)
                feature_msg.data = np.append(feature.findholecentre(bw_img), (yaw, pitch, yaw, pitch))

                write_bag.write(camera_topic, camera_msg,  t)
                write_bag.write(joy_topic, joy_msg,t)
                write_bag.write(record_topic, feature_msg, t)
                camera_msg = None
                joy_msg = None
                
        bag.close()
        write_bag.close()
    f.close()
