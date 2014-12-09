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
from std_msgs.msg import Float32MultiArray

from cv_bridge import CvBridge, CvBridgeError
import cv2
import rosbag

class BagSaver:
    def __init__(self):
        self.joy_topic = rospy.get_param('~joy_topic', default='/cmd_vel')
        self.img_topic = rospy.get_param('~img_topic', default='/camera/image_raw')
        self.execute_topic = rospy.get_param('~execute_topic', default='/execute')
        self.bag_file_path = rospy.get_param('~bag_file_path', default='/home/icoderaven/test'+str(rospy.Time.now().secs)+'.bag')
        self.dagger_active = rospy.get_param('~dagger_active', default=False)
        if self.dagger_active:
            self.array_topic = rospy.get_param('~array_topic', default='/record')
        self.fix_image = rospy.get_param('~fix_image', default=True)
        
        self.last_joy_msg = None
        self.last_img_msg = None
        if self.dagger_active:
            self.last_array_msg = None
        self.last_time = rospy.Time.now()
        
        self.bag_file = rosbag.Bag(self.bag_file_path, 'w')
        
        self.bridge = CvBridge()
        
	self.publisher = rospy.Publisher('/my_test', Image,queue_size=1);         
	self.joy_sub = rospy.Subscriber(self.joy_topic, Twist, self.joy_update, queue_size=1)
        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_update, queue_size=1)
        self.empty_sub = rospy.Subscriber(self.execute_topic, Empty, self.execute_update, queue_size=1)
        if self.dagger_active:
            self.array_sub = rospy.Subscriber(self.array_topic, Float32MultiArray, self.array_update, queue_size=1)  
        
    
    def __del__(self):
        self.bag_file.close()
    
    def close(self):
        self.bag_file.close()    
    def joy_update(self, data):
        self.last_joy_msg = data
        
    def img_update(self, data):
        '''If the input image is garbled because of UVC camera shenanigans, fix the encoding'''
        if self.fix_image == True:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            #Convert from yuv2 to rgb
            new_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            new_msg = self.bridge.cv2_to_imgmsg(new_image, encoding='bgr8')
            if self.last_img_msg is None and (rospy.Time.now() - self.last_time).to_sec() > 4:
                self.last_img_msg = new_msg
            if self.last_img_msg is not None:
                self.publisher.publish(self.last_img_msg)
        else:
            self.last_img_msg = data
    
    def array_update(self, data):
        '''Get the data as a numpy array'''
        self.last_array_msg = data
        
    def execute_update(self, data):
        '''Write the latest two messages to the bagfile'''
        if (self.last_img_msg is not None) and (self.last_joy_msg is not None) and (self.last_array_msg is not None):
            self.write_bag()
            self.publisher.publish(self.last_img_msg)
            self.last_img_msg = None
            self.last_joy_msg = None
            self.last_array_msg = None
            return True
        else:
            rospy.loginfo('[BagSaver] Message tuple not received')
            return False 
    
        
    def write_bag(self):
	    #Only if it has been 3 seconds since we last recorded a data point
        if (rospy.Time.now() - self.last_time).to_sec() > 3:
	    rospy.loginfo('[BagWriter] Adding data tuple')
            self.bag_file.write(self.joy_topic, self.last_joy_msg)
            self.bag_file.write(self.img_topic, self.last_img_msg)
            self.bag_file.write(self.array_topic, self.last_array_msg)
	    self.last_time = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('bag_file_saver')
    my_saver = BagSaver()
    rospy.spin()
    my_saver.close()
