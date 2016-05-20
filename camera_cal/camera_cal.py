# -*- coding: utf-8 -*-
"""
Created on Wed Feb 10 22:22:15 2016

@author: perrytsao
"""
import numpy as np
import cv2
import glob


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((54,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray',gray)
    #cv2.waitKey(5000)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None)
    
    
    #[isFound, centers] = cv2.findCirclesGrid(image, shape, flags = cv2.CALIB_CB_ASYMMETRIC_GRID + cv2.CALIB_CB_CLUSTERING)
    #shape=(4,11)
    #[ret, corners] = cv2.findCirclesGrid(gray, shape, flags = cv2.CALIB_CB_ASYMMETRIC_GRID + cv2.CALIB_CB_CLUSTERING)    
    
    # If found, add object points, image points (after refining them)
    if ret == True:
        print "found: "+fname        
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(5000)
    else:
        print "not found: "+fname

cv2.destroyAllWindows()


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

for xname in images:
    img = cv2.imread(xname)
    h,  w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
    
    # undistort
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    
    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite('calibresult.png',dst)
    cv2.imshow(xname,dst)
    cv2.waitKey(5000)

from datetime import datetime
timestamp="{:%Y_%m_%d_%H_%M}".format(datetime.now())

np.savez('camera_cal_data_'+timestamp, newcameramtx=newcameramtx, roi=roi, mtx=mtx, dist=dist)

cv2.destroyAllWindows()
