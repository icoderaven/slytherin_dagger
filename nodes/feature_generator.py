#!/usr/bin/env python
'''
Simple utility to listen to the Execute button and save the latest image and joystick control input and save them as bag files
'''
import roslib

roslib.load_manifest('slytherin_dagger')
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class FeatureGenerator:
    def __init__(self):
        self.img_topic = rospy.get_param('~img_topic', default='/camera/image_raw')
        self.feat_topic = rospy.get_param('~feat_topic', default='/feature')
        self.img_sub = rospy.Subscriber(self.img_topic, Image, self.img_update, queue_size=1)
        self.feat_pub = rospy.Publisher(self.feat_topic, Float32MultiArray, queue_size=1)
        self.bridge = CvBridge()

    # given an image this code find the centroid of the hole in the image
    def findholecentre(self, original):
        #############################################
        #find a mask that removes the black empty area
        ##############################################
        #original = cv2.imread('filename.png',0)


        #cv2.imshow('original', original)
        #cv2.waitKey(-1)

        retval, image = cv2.threshold(original, 70, 70, cv2.THRESH_BINARY)
        #cv2.imshow('thresh', image)
        #cv2.waitKey(-1)

        ##################################################
        # we erode the image to remove stray white pixels
        ##################################################
        kernel = np.ones((5, 5), np.uint8)
        erosion2 = cv2.erode(image, kernel, iterations=1)
        #cv2.imshow('eroded final image', erosion2)
        #cv2.waitKey(-1)

        ########################################
        #Centroid calculation from binary image
        ########################################
        M = cv2.moments(erosion2, binaryImage=True)

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        centroid = [cx, cy]
        arr1 = np.array(centroid)

        ######################################################################
        #we plot a small circle around the centroid for visualization purpose
        ######################################################################
        cv2.circle(original, (cx, cy), 10, (255, 0, 0), -1)
        #cv2.imshow('final image', original)
        cv2.waitKey(-1)
        return arr1

    def img_update(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        bw_img = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        feat = self.findholecentre(bw_img)
        msg = Float32MultiArray()
        msg.data = feat
        self.feat_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('feature_generator')
    my_saver = FeatureGenerator()
    rospy.spin()
