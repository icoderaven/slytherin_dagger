#!/usr/bin/env python
'''
Simple utility to listen to the Execute button and save the latest image and joystick control input and save them as bag files
'''
import roslib
roslib.load_manifest('slytherin_dagger')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image

import rosbag.Bag

class BagSaver:
    def __init__(self):
        self.joy_topic = rospy.get_param('~joy_topic', default='\cmd_vel')
        self.img_topic = rospy.get_param('~img_topic', default='\camera\image_raw')
        self.execute_topic = rospy.get_param('~execute_topic', default='\execute')
        self.bag_file_path = rospy.get_param('~bag_file_path')
        
        self.last_joy_msg = None
        self.last_img_msg = None
        
        self.bag_file = rosbag.Bag(self.bag_file_path, 'w')
        
        self.joy_sub = rospy.Subscriber(self.joy_topic, Twist, self.joy_update, queue_size=1)
        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_update, queue_size=1)
        self.empty_sub = rospy.Subscriber(self.execute_topic, Empty, self.execute_update, queue_size=1)  
    
    def __del__(self):
        self.bag_file.close()
        
    def joy_update(self, data):
        self.last_joy_msg = data
        
    def img_update(self, data):
        self.last_img_msg = data
        
    def execute_update(self, data):
        '''Write the latest two messages to the bagfile'''
        if self.last_img_msg is not None and self.last_joy_msg is not None:
            rospy.loginfo('[BagSaver] Message tuple not received')
            return False 
        
        else:
            self.write_bag()
            self.last_img_msg = None
            self.last_joy_msg = None
            return True
        
    def write_bag(self):
        self.bag_file.write(self.joy_topic, self.last_joy_msg)
        self.bag_file.write(self.img_topic, self.last_img_msg)

if __name__ == '__main__':
    rospy.init_node('bag_file_saver')
    my_saver = BagSaver()
    rospy.spin()