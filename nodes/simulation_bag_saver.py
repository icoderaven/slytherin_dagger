#!/usr/bin/env python
'''
Simple utility to listen to record_simul_bag topic to which matlab publishes and save it to a bag file
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
        self.feat_topic = rospy.get_param('feat_topic', default='record_simul_bag')
        self.bag_file_path = rospy.get_param('~bag_file_path',
                                             default='/media/Obelix/GraduateSchool/FallCourses14/16831/project/snake_robot_project/src/slytherin_dagger/Simulation_bags/it0_bags/'+str(rospy.Time.now().secs)+'.bag')

        self.last_time = rospy.Time.now()

        self.bag_file = rosbag.Bag(self.bag_file_path, 'w')

        self.feat_sub = rospy.Subscriber(self.feat_topic, Float32MultiArray, self.write_bag, queue_size=1)

    def __del__(self):
        self.bag_file.close()

    def close(self):
        self.bag_file.close()

    def write_bag(self,msg):
        rospy.loginfo('[BagWriter] Adding data tuple')
        self.bag_file.write(self.feat_topic, msg)


if __name__ == '__main__':
    rospy.init_node('simulation_bag_saver')
    rospy.loginfo('Started BagSaver')
    my_saver = BagSaver()
    rospy.spin()
    my_saver.close()
