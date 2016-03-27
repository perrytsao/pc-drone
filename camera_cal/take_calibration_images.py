# -*- coding: utf-8 -*-
"""
This utility takes snapshots from the webcam
You may need to edit the camera channel, width, height, 

Created on Mon Feb 08 23:00:39 2016

@author: perrytsao 
More info at www.makehardware.com/webcam-latency

"""
import timeit
import time
import cv2

###############################################
width=640
height=480
fps=30
camera_channel=1
fname="ca_image"
###############################################
wait_time=1

cv2.namedWindow("preview")
vc = cv2.VideoCapture(camera_channel)
vc.set(cv2.CAP_PROP_FRAME_WIDTH,width)
vc.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
vc.set(cv2.CAP_PROP_FPS,fps) 

################################################
time.sleep(2)
font = cv2.FONT_HERSHEY_SIMPLEX
try: 
    if vc.isOpened(): # try to get the first frame
        rval, frame_o = vc.read()
        
    else:
        rval = False
    ii=100
    toc=0    
    tic=timeit.default_timer()
    while rval:
        toc_old=toc
        toc=timeit.default_timer()        
        delta=toc-toc_old
        print("delta: %0.3f  fps: %0.3f" % (delta, 1/delta))
        #cv2.putText(frame_o, "%0.3f" % (toc-tic), (50,200), font, 2, (255,255,255),4,cv2.LINE_AA)       
        cv2.imshow("preview", frame_o)
        key = cv2.waitKey(wait_time)
      
        ## Monitor keyboard
        if key == 27: # exit on ESC
            break
        elif key == 32:
            cv2.imwrite(fname+str(ii)+".jpg", frame_o)
            ii+=1
        rval, frame_o = vc.read()
          
finally: 
    vc.release()
    cv2.destroyAllWindows()
    
