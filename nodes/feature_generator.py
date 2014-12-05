#!/usr/bin/env python
'''
Simple utility to listen to the Execute button and save the latest image and joystick control input and save them as bag files
'''
import roslib

roslib.load_manifest('slytherin_dagger')
import rospy
import cv2
import numpy as np
import math
import pdb
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
    def img_update(self, data):
        #rospy.loginfo("received image")
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        bw_img = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        feat = findholecentre(bw_img)
        msg = Float32MultiArray()
        msg.data = feat
        self.feat_pub.publish(msg)

def findholecentre(original):
    #############################################
    #find a mask that removes the black empty area
    ##############################################
    #original = cv2.imread('filename.png',0)
    #cv2.imshow('original', original)
    #cv2.waitKey(-1)
    duplicate=original.copy()
    image1 = original
    #image1 = cv2.medianBlur(original, 3)
    lo = 3
    hi = 3
    kernel = np.ones((5, 5), np.uint8)
    kernel2 = np.ones((3, 3), np.uint8)
    h,w = image1.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)

    cv2.floodFill(image1,mask, (1,1),(255,255,255),(lo,)*3, (hi,)*3)
    image1 = cv2.dilate(image1, kernel, iterations=3)
    #image1[image1==255]=0
    #cv2.imshow('floodfill image', image1)
    #cv2.imshow('mask1', mask)
    #cv2.waitKey(-1)
    retval, image = cv2.threshold(image1, 254, 255, cv2.THRESH_BINARY_INV)
    #image = cv2.adaptiveThreshold(original, 255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,51,2)
    #cv2.imshow('thresh', image)
    #cv2.waitKey(-1)
    #######################################################
    temp = image.copy()        
    contours,hierarchy = cv2.findContours(temp,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #print cv2.boundingRect(contours)
    ##pdb.set_trace()        
    cnt=None
    for c in contours:
        cv2.drawContours(image,[c],0,255,-1)
        boundingbox=cv2.boundingRect(c)
        if boundingbox[2]*boundingbox[3]<1000:
            continue
        cnt=c				
        break
    boundingbox= cv2.boundingRect(cnt)
    centreBoundingBox=[boundingbox[0]+boundingbox[2]/2,boundingbox[1]+boundingbox[3]/2]
    #boxsize=boundingbox[2]
    #cv2.imshow('contours', temp) #temp is the mask
    #cv2.imshow('mask after contours', image) 
    #cv2.waitKey(-1)
    M1 = cv2.moments(cnt)#, binaryImage=True)
    cx1 = int(M1['m10'] / (M1['m00'] + np.finfo(float).eps))
    cy1 = int(M1['m01'] / (M1['m00'] + np.finfo(float).eps))
    centroidimg = [cx1, cy1]#centroid of the mask
    radius=math.sqrt(M1['m00']/3.14)
    #tmp2 = original.copy()
    #cv2.circle(tmp2, (cx1, cy1), 10, (255, 0, 0), -1)
    ##cv2.imshow('final image', original)
    ##cv2.waitKey(-1)

    #####################################################################
    #we plot a small circle around the centroid of mask for visualization purpose
    ######################################################################
    #apply the mask on the original image after inverse thresholding it
    ######################pr#####################
    temp= original.copy()
    res = cv2.bitwise_and(image, temp)#res is the image after masking
    #cv2.imshow('maskedimage', res)
    #cv2.waitKey(-1)
    ########################################
    #Finding centroid of dark region in the image
    ########################################
    M2 = cv2.moments(res, binaryImage=True)
    cx2 = int(M2['m10'] / (M2['m00'] + np.finfo(float).eps))
    cy2 = int(M2['m01'] / (M2['m00'] + np.finfo(float).eps))
    centroid2 = [cx2-cx1, cy2-cy1]
    arr1 = np.array(centroid2)
    ############################################
    #find average intensity in 4 quadrants around the circle
    divisions=4
    steph=boundingbox[2]/divisions
    stepv=boundingbox[3]/divisions
    mean=np.zeros((divisions,divisions))
    for row in range(divisions):
        for column in range(divisions):
	    subset=res[boundingbox[1]+row*steph:boundingbox[1]+(row+1)*steph,boundingbox[0]+column*stepv:boundingbox[0]+(column+1)*stepv]		
	    mean[row,column]=cv2.mean(subset)[0]
    mean=mean/np.max(mean)		
    #pdb.set_trace()	

    #imagetl=duplicate[cx1-radius:cx1,cy1-radius:cy1]
    #imagetr=duplicate[cx1-radius:cx1,cy1:cy1+radius]
    #imagebl=duplicate[cx1:cx1+radius,cy1-radius:cy1]
    #imagebr=duplicate[cx1:cx1+radius,cy1:cy1+radius]
    ##cv2.imshow('image segment top left', imagetl)
    ##cv2.imshow('image segment top right', imagetr)
    ##cv2.imshow('image segment bottom left', imagebl)
    ##cv2.imshow('image segment bottom right', imagebr)
    ##cv2.waitKey(-1)
    #meantl=cv2.mean(imagetl)
    #meantr=cv2.mean(imagetr)
    #meanbl=cv2.mean(imagebl)
    #meanbr=cv2.mean(imagebr)
    #print meantl[0]
    #print meantr[0]
    #print meanbl[0]
    #print meanbr[0]
    ###########################################        
    #sobelx = cv2.Sobel(res,cv2.CV_64F,1,0,ksize=5)
    #M3 = cv2.moments(sobelx[cx1-radius:cx1,cy1-radius:cy1], binaryImage=True)
    #cx3 = int(M3['m10'] / (M3['m00'] + np.finfo(float).eps))
    #cy3 = int(M3['m01'] / (M3['m00'] + np.finfo(float).eps))
    #centroidxtl = [cx3, cy3]
    #M3 = cv2.moments(sobelx[cx1-radius:cx1,cy1:radius+cy1], binaryImage=True)
    #cx3 = int(M3['m10'] / (M3['m00'] + np.finfo(float).eps))
    #cy3 = int(M3['m01'] / (M3['m00'] + np.finfo(float).eps ))
    #centroidxtr = [cx3, cy3]
    #M3 = cv2.moments(sobelx[cx1:cx1+radius,cy1-radius:cy1], binaryImage=True)
    #cx3 = int(M3['m10'] / (M3['m00'] + np.finfo(float).eps))
    #cy3 = int(M3['m01'] / (M3['m00'] + np.finfo(float).eps))
    #centroidxbl = [cx3, cy3]
    #M3 = cv2.moments(sobelx[cx1:radius+cx1,cy1:radius+cy1], binaryImage=True)
    #cx3 = int(M3['m10'] / (M3['m00'] + np.finfo(float).eps))
    #cy3 = int(M3['m01'] / (M3['m00']+ np.finfo(float).eps))
    #centroidxbr = [cx3, cy3]
    ##cv2.imshow('gradient image x',sobelx[cx1-radius:cx1,cy1-radius:cy1])
    ##cv2.waitKey(-1)
    ##########################################
    #sobely = cv2.Sobel(res,cv2.CV_64F,0,1,ksize=5)
    #M3 = cv2.moments(sobely[cx1-radius:cx1,cy1-radius:cy1], binaryImage=True)
    #cx3 = int(M3['m10'] / (M3['m00'] + np.finfo(float).eps))
    #cy3 = int(M3['m01'] / (M3['m00'] +np.finfo(float).eps))
    #centroidytl = [cx3, cy3]
    #M3 = cv2.moments(sobelx[cx1-radius:cx1,cy1:radius+cy1], binaryImage=True)
    #cx3 = int(M3['m10'] / (M3['m00'] + np.finfo(float).eps))
    #cy3 = int(M3['m01'] / (M3['m00'] + np.finfo(float).eps))
    #centroidytr = [cx3, cy3]
    #M3 = cv2.moments(sobelx[cx1:cx1+radius,cy1-radius:cy1], binaryImage=True)
    #cx3 = int(M3['m10'] / (M3['m00'] + np.finfo(float).eps))
    #cy3 = int(M3['m01'] / (M3['m00'] + np.finfo(float).eps))
    #centroidybl = [cx3, cy3]
    #M3 = cv2.moments(sobelx[cx1:radius+cx1,cy1:radius+cy1], binaryImage=True)
    #cx3 = int(M3['m10'] / (M3['m00'] + np.finfo(float).eps))
    #cy3 = int(M3['m01'] / (M3['m00'] + np.finfo(float).eps))
    #centroidybr = [cx3, cy3]
    #countx = cv2.countNonZero(sobelx)
    #county = cv2.countNonZero(sobely)
    ##########################################
    #print radius
    #cv2.rectangle(res,(boundingbox[0],boundingbox[1]),(boundingbox[0]+boundingbox[2],boundingbox[1]+boundingbox[3]),(255,255,255))
    ##cv2.imshow('subset of image', res[boundingbox[1]:boundingbox[1]+boundingbox[3],boundingbox[0]:boundingbox[0]+boundingbox[2]])
    ##cv2.waitKey(-1)
    #print res[cx1-radius:cx1+radius-1,cy1-radius:cy1+radius-1].shape
    #hog = cv2.HOGDescriptor((256,256), (256,256), (256,256), (128,128 ), 4)
    #desc = hog.compute( res[boundingbox[1]:boundingbox[1]+boundingbox[3],boundingbox[0]:boundingbox[0]+boundingbox[2]] )
    #hog = cv2.HOGDescriptor()
    #desc = hog.compute(res,hog.blockStride, hog.cellSize,((boundingbox[0],boundingbox[1]),))
    #print desc.size
    #print boundingbox
    #print [cx1,cy1]
    #aw=np.concatenate((desc[4:4*3],desc[4*4:4*12],desc[4*13:4*15]))#HOG features for 16 boxes minus 4 corners
    ##pdb.set_trace()
    return mean.flatten()



if __name__ == '__main__':
    rospy.init_node('feature_generator')
    my_saver = FeatureGenerator()
    rospy.spin()
