# -*- coding: utf-8 -*-
"""
Created on Mon Feb 08 23:00:39 2016

@author: perrytsao
"""

import cv2
import numpy as np
import itertools

from datetime import datetime
timestamp="{:%Y_%m_%d_%H_%M}".format(datetime.now())

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
params.minArea = 5
 
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
    lower_green = np.array([60,20,20])
    upper_green = np.array([95,255,255])
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
            keypoints_side=[keypoints[i] for i in [leftpt]]
            im_with_keypoints1 = cv2.drawKeypoints(im_with_midpoint_frontpoint, keypoints_side, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            keypoints_side=[keypoints[i] for i in [rightpt]]
            im_with_keypoints = cv2.drawKeypoints(im_with_keypoints1, keypoints_side, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            textstr="%0.4f dist. %i,%i center" % (max_dist_val, keypoints[middlepoint].pt[0], keypoints[middlepoint].pt[1])
            max_blob_dist=max_dist_val
            blob_center=keypoints[middlepoint].pt 
            keypoints[middlepoint].pt[1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(im_with_keypoints, textstr,(10,50), font, .8,(255,255,255),2,cv2.LINE_AA)
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        #im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        else:
            im_with_keypoints=crop_frame    
            print "%i blobs" % (len(keypoints))
            max_blob_dist=None
            blob_center=None
    else:
        print "no blobs"
        im_with_keypoints=crop_frame
        max_blob_dist=None
        blob_center=None
    return im_with_keypoints, max_blob_dist, blob_center #, keypoint_in_orders


###############################################
cv2.namedWindow("preview")
vc = cv2.VideoCapture(1)

fname="drone_green_dot"
width=800
height=600
wait_time=33
fps=int(1/.001/(float(wait_time)))
vc.set(3,width)
vc.set(4,height)
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter('output13'+timestamp+'.avi',fourcc, 30.0, (width,height),1)

import serial, time, msvcrt

throttle=1000
aileron=1500 # moves left/right
elevator=1500 #moves front back
rudder=1500 # yaw, rotates the drone

tg=5
ag=1
eg=1
rg=1

zpos=50
xypos=(350, 250)
command=""
start_flying=0
no_position_cnt=0

oldz=45
oldx=350
oldy=250
dz=0
dx=0
dy=0
xspeed=0
yspeed=0
zspeed=0
speeds=""

font = cv2.FONT_HERSHEY_SIMPLEX
try: 
    arduino=serial.Serial('COM3', 115200, timeout=.01)
    time.sleep(1) #give the connection a second to settle    
    
    if vc.isOpened(): # try to get the first frame
        rval, frame_o = vc.read()
        #frame_undistort=undistort_crop(np.rot90(frame_o, 2))
        frame_undistort=undistort_crop(frame_o)   
        frame, zpos, xypos=add_blobs(frame_undistort)
    else:
        rval = False
    ii=100
    while rval:
        
        cv2.imshow("preview", frame)
        
        key = cv2.waitKey(wait_time)
        rval, frame_o = vc.read()
        #frame_undistort=undistort_crop(np.rot90(frame_o, 2))
        frame_undistort=undistort_crop(frame_o)
        frame, zpos, xypos=add_blobs(frame_undistort)
        
        ## Serial comms
        data = arduino.readline()
        while data:
            print "[AU]: "+data.rstrip("\n") #strip out the new lines for now
            # (better to do .read() in the long run for this reason    
            data=arduino.readline()

        throttle=min(throttle, 2000)
        aileron=min(aileron, 1510)
        elevator=min(elevator, 1510)
        rudder=min(rudder, 1600)        
        
        throttle=max(throttle, 1000)
        aileron=max(aileron, 1490)
        elevator=max(elevator, 1490)
        rudder=max(rudder, 1400)               
        
        
        
        command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)
        print "[PC]: "+command
        arduino.write(command+"\n")
        try:
            dz=(zpos-oldz)*0.3+0.7*dz
            dx=(xypos[0]-oldx)*0.3+0.7*dx
            dy=(xypos[1]-oldy)*0.3+0.7*dy
    
            oldz=zpos
            oldx=xypos[0]
            oldy=xypos[1]        
        except:
            print "no speed"
        speeds="dz: %0.2f dx: %0.2f dy: %0.2f" % (dz, dx, dy)     
        targets="tsz: %0.2f tsx: %0.2f tsy: %0.2f" % (zspeed, xspeed, yspeed)     
        cv2.putText(frame, command,(10,100), font, .8,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame, speeds,(10,150), font, .8,(255,255,255),2,cv2.LINE_AA)        
        cv2.putText(frame, targets,(10,200), font, .8,(255,255,255),2,cv2.LINE_AA) 
        if start_flying:
            
            frame_pad=cv2.copyMakeBorder(frame,111,0,86,00,cv2.BORDER_CONSTANT,value=[255,0,0])
            out.write(frame_pad)
            try: 
                print "Zpos: %i Xpos: %i Ypos: %i" % (zpos, xypos[0], xypos[1])

                

                
                # compare to target speed               
                if dz > zspeed:
                    throttle-=tg
                else:
                    throttle+=tg
                
                if dx > xspeed:
                    aileron-=ag
                else:
                    aileron+=ag
                    
                if dy > yspeed:
                    elevator-=eg
                else:
                    elevator+=eg                
            
                clamp=lambda n, minn, maxn: (max(min(maxn, n), minn))
                if zpos > 60:
                    print "highalt"
                    aileron=clamp(aileron, 1495, 1510)
                    elevator=clamp(elevator, 1495, 1510)
                else: 
                    print "lowalt"
                    aileron=clamp(aileron, 1499, 1501)
                    elevator=clamp(elevator, 1499, 1501)
                # set target speeds                
                if zpos > 65:
                    zspeed=-.1
                else:
                    zspeed=+.1
                if xypos[0] > 350:
                    xspeed=-0.2 # move left
                else:
                    xspeed=0.2 # move right
                if xypos[1] > 250:
                    yspeed=-0.2 # move up
                else:
                    yspeed=+0.2 # move down
                    
                no_position_cnt=0    
            except:
                no_position_cnt+=1
                print "no position"
                if no_position_cnt>15:
                    throttle=1000
                    start_flying=0  
        ## Monitor keyboard
        if key == 27: # exit on ESC
            break
        elif key == 32:
            cv2.imwrite(fname+str(ii)+".jpg", frame)
            ii+=1
        elif key == 119: #w
            throttle=1200
            aileron=1500 # turns left
            elevator=1500
            rudder=1500 # yaw, rotates the drone
            start_flying=1
            print "START FLYING"
        elif key == 115: #s
            throttle=1000
            start_flying=0
            
            
            
finally:
    # close the connection
    arduino.close()
    # re-open the serial port which will w for Arduino Uno to do a reset
    # this forces the quadcopter to turn off. 
    arduino=serial.Serial('COM3', 115200, timeout=.01)
    arduino.close()
    # close it again so it can be reopened the next time it is run.      
    
    vc.release()
    cv2.destroyWindow("preview")
    out.release()
