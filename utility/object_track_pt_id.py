# -*- coding: utf-8 -*-
"""
Created on Tue Feb 09 06:32:39 2016

@author: perrytsao
"""

import numpy as np
import cv2
import glob
import itertools

calfile=np.load('camera_cal_data_2016_02_20_03_09.npz')

newcameramtx=calfile['newcameramtx']
roi=calfile['roi']
mtx=calfile['mtx']
dist=calfile['dist']

images = glob.glob('drone*.jpg')

for fname in images:
    orig_img = cv2.imread(fname)
    
    #undistort and crop
    dst = cv2.undistort(orig_img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi
    crop_frame = dst[y:y+h, x:x+w]    
    #crop_frame=dst[x:x+h, y:y+w]    
    
    frame=cv2.GaussianBlur(crop_frame, (3, 3), 0)
        
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # define range of green color in HSV
    lower_green = np.array([70,50,50])
    upper_green = np.array([85,255,255])
    
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.erode(mask, None, iterations=1)
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
    params.minInertiaRatio = 0.01
     
   

    detector = cv2.SimpleBlobDetector_create(params)
 
    # Detect blobs.
    reversemask=255-mask
    keypoints = detector.detect(reversemask)
    if keypoints:
        print "found blobs"
        if len(keypoints) > 4:
            keypoints.sort(key=(lambda s: s.size))
            keypoints=keypoints[0:3]
        if len(keypoints)==4:
            pts= np.array([keypoints[i].pt for i in range(4)])
            #x,y=zip(*pts)
            
            # Calculate distances between all combinations of points
            dis_vectors = [l - r for l, r in itertools.combinations(pts, 2)]
            dcalc=[np.linalg.norm(dis_vectors[i]) for i in range(6)]

            # find the closest point to all of them, that is the middle point
            mean_a=np.array([dcalc[i] for i in [0,1,2]]).sum()/4.0
            mean_b=np.array([dcalc[i] for i in [0,3,4]]).sum()/4.0
            mean_c=np.array([dcalc[i] for i in [1,3,5]]).sum()/4.0
            mean_d=np.array([dcalc[i] for i in [2,4,5]]).sum()/4.0
            middlepoint=np.argmin(np.array([mean_a, mean_b, mean_c, mean_d]))

            idx=np.argmax(dcalc) # find two furthest points, those are left and right sidepoints
            max_dist_val=np.max(dcalc)            
            print max_dist_val
            if idx ==0:
                sidepts=[0,1]
            elif idx==1:
                sidepts=[0,2]        
            elif idx==2:
                sidepts=[0,3]  
            elif idx==3:
                sidepts=[1,2]                  
            elif idx==4:
                sidepts=[1,3]                  
            elif idx==5:
                sidepts=[2,3]            
                
            # the frontpoint is the remaining one.
            frontpoint=6-np.array(sidepts+[middlepoint]).sum()    
            
            # now determine which side point is the left one
            # http://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
            a=keypoints[middlepoint].pt
            b=keypoints[frontpoint].pt
            c=keypoints[sidepts[0]].pt
            if ((b[0] - a[0])*(c[1] - a[1]) - (b[1] - a[1])*(c[0] - a[0])) < 0:
                leftpt=sidepts[0]
                rightpt=sidepts[1]
            else:
                leftpt=sidepts[1]
                rightpt=sidepts[0]
            
            
            im_with_midpoint = cv2.drawKeypoints(frame, [keypoints[middlepoint]], np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            im_with_midpoint_frontpoint = cv2.drawKeypoints(im_with_midpoint, [keypoints[frontpoint]], np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


            
            #keypoints_side=[keypoints[i] for i in sidepts]
            keypoints_side=[keypoints[i] for i in [leftpt]]
            im_with_keypoints1 = cv2.drawKeypoints(im_with_midpoint_frontpoint, keypoints_side, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    
            
            keypoints_side=[keypoints[i] for i in [rightpt]]
            im_with_keypoints = cv2.drawKeypoints(im_with_keypoints1, keypoints_side, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            textstr="%0.4f dist. %i,%i center" % (max_dist_val, keypoints[middlepoint].pt[0], keypoints[middlepoint].pt[1])
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(im_with_keypoints, textstr,(10,50), font, .8,(255,255,255),2,cv2.LINE_AA)
            #cv2.putText(im_with_keypoints, textstr, font)
            cv2.imshow("Keypoints", im_with_keypoints) 
            
            
            
    else:
        print "no blobs"
 
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
     
      
    
    
    
    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)     
    # Show keypoints
    
        
    k = cv2.waitKey(5000)

cv2.destroyAllWindows()
