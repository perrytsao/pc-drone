# -*- coding: utf-8 -*-
"""
Created on Mon Feb 08 23:00:39 2016

@author: perrytsao
"""

import cv2
import numpy as np

# load params to undistort images
calfile=np.load('camera_cal_data_2016_02_20_03_09.npz')
newcameramtx=calfile['newcameramtx']
roi=calfile['roi']
mtx=calfile['mtx']
dist=calfile['dist']

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

def undistort_crop(orig_img):
    #undistort and crop
    dst = cv2.undistort(orig_img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi
    crop_frame = dst[y:y+h, x:x+w]    
    return crop_frame
    
def add_blobs(crop_frame):
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
    detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs.
    reversemask=255-mask
    keypoints = detector.detect(reversemask)
    if keypoints:
        print "found blobs"
        if len(keypoints) > 4:
            keypoints.sort(key=(lambda s: s.size))
            keypoints=keypoints[0:3]
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    else:
        print "no blobs"
        im_with_keypoints=crop_frame
        
    return im_with_keypoints #, max_blob_dist, blob_center, keypoint_in_orders

cv2.namedWindow("preview")
vc = cv2.VideoCapture(1)

fname="drone_green_dot"
width=800
height=600
wait_time=33
fps=int(1/.001/(float(wait_time)))
vc.set(3,width)
vc.set(4,height)


if vc.isOpened(): # try to get the first frame
    rval, frame_o = vc.read()
    frame_undistort=undistort_crop(np.rot90(frame_o, 2))
    frame=add_blobs(frame_undistort)
else:
    rval = False
ii=100
while rval:
    cv2.imshow("preview", frame)
    key = cv2.waitKey(wait_time)
    rval, frame_o = vc.read()
    frame_undistort=undistort_crop(np.rot90(frame_o, 2))
    frame=add_blobs(frame_undistort)
    
    if key == 27: # exit on ESC
        break
    if key == 32:
        cv2.imwrite(fname+str(ii)+".jpg", frame)
        ii+=1
vc.release()
cv2.destroyWindow("preview")
#out.release()