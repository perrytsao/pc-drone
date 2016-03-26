# -*- coding: utf-8 -*-
"""
Created on Mon Feb 08 23:00:39 2016

@author: perrytsao
"""

import cv2
import numpy as np

cv2.namedWindow("preview")
vc = cv2.VideoCapture(1)

fname="drone_green_dot_USBFHD01M"
width=640
height=480
wait_time=33
fps=int(1/.001/(float(wait_time)))
vc.set(3,width)
vc.set(4,height)


if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
    #frame=np.rot90(frame_o, 2)
else:
    rval = False
ii=100
while rval:
    cv2.imshow("preview", frame)
    key = cv2.waitKey(wait_time)
    rval, frame = vc.read()
    #frame=np.rot90(frame_o, 2)
    #out.write(frame)
    
    if key == 27: # exit on ESC
        break
    if key == 32:
        cv2.imwrite(fname+str(ii)+".jpg", frame)
        ii+=1
vc.release()
cv2.destroyWindow("preview")
#out.release()