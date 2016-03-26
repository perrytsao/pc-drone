# -*- coding: utf-8 -*-
"""
Created on Tue Feb 09 06:32:39 2016

@author: perrytsao
"""

import numpy as np
import cv2
import glob
import itertools

calfile=np.load('camera_cal_data_2016_03_25_15_23.npz')

newcameramtx=calfile['newcameramtx']
roi=calfile['roi']
mtx=calfile['mtx']
dist=calfile['dist']

images = glob.glob('drone_green_dot_USBFHD01M*.jpg')

for fname in images:
    print fname
    orig_img = cv2.imread(fname)
    
    #undistort and crop
    dst = cv2.undistort(orig_img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi
    crop_frame = dst[y:y+h, x:x+w]    
    
    frame=cv2.GaussianBlur(crop_frame, (3, 3), 0)
        
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # define range of green color in HSV
    lower_green = np.array([60,20,20])
    upper_green = np.array([85,255,255])
    
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=1)    
    
    
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    

 
    # Set up the detector with default parameters.
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
     
    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 256;
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 30
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5
     
    # Filter by Inertia
    params.filterByInertia =True
    params.minInertiaRatio = 0.5
     
   

    detector = cv2.SimpleBlobDetector_create(params)
 
    # Detect blobs.
    reversemask=255-mask
    keypoints = detector.detect(reversemask)
    if keypoints:
        print "found blobs"
        if len(keypoints) > 4:
            keypoints.sort(key=(lambda s: s.size))
            keypoints=keypoints[0:3]
            
    
            #front_point=
            
            #public bool isLeft(Point a, Point b, Point c){
            #    return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x)) > 0;

    else:
        print "no blobs"
 
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
     
      
    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    
    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)     
    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)            
        
    k = cv2.waitKey(10000)

cv2.destroyAllWindows()
