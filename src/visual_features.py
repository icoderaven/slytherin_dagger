#!/usr/bin/env python

import cv2
import numpy as np


def findholecentre(original):

    retval, image = cv2.threshold(original,40, 255, cv2.THRESH_BINARY)

    # cv2.imshow('thresh', image)
    # cv2.waitKey(-1)


    contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    temp = original.copy()

    for cnt in contours:
        cv2.drawContours(temp, [cnt], 0, 255, -1)
    #cv2.imshow('contours', temp) #temp is the mask
    #cv2.waitKey(-1)

    ###########################################################################
    #here we apply the mask on the original image after inverse thresholding it
    ###########################################################################
    retval, thresh1 = cv2.threshold(original, 40, 255, cv2.THRESH_BINARY_INV)
    #cv2.imshow('inverted thresh1', thresh1)
    #cv2.waitKey(-1)

    res = cv2.bitwise_and(thresh1, temp)
    #cv2.imshow('maskedimage', res)
    #cv2.waitKey(-1)
    temp4 = res.copy()
    ##################################################
    # we erode the image to remove stray white pixels
    ##################################################
    kernel = np.ones((5, 5), np.uint8)
    erosion2 = cv2.erode(res, kernel, iterations=1)
    #cv2.imshow('eroded final image', erosion2)
    #cv2.waitKey(-1)

    ########################################
    #Centroid calculation from binary image
    ########################################
    M = cv2.moments(erosion2, binaryImage=True)

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    feat_array = np.array([cx,cy])

    return feat_array
