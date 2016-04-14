# -*- coding: utf-8 -*-
"""
Created on Mon Feb 08 23:00:39 2016

@author: perrytsao
"""

import cv2
import numpy as np

cv2.namedWindow("preview")
vc = cv2.VideoCapture(1)

fname="purple_apr_expauto_"
#fname="dark_light_USBFHD01M"
width=640
height=480
wait_time=33
fps=int(1/.001/(float(wait_time)))
vc.set(3,width)
vc.set(4,height)
print vc.get(cv2.CAP_PROP_EXPOSURE)
print vc.get(cv2.CAP_PROP_AUTO_EXPOSURE)
vc.set(cv2.CAP_PROP_SETTINGS, 0) # show camera properties dialog
#vc.set(cv2.CAP_PROP_EXPOSURE,-10)
#vc.set(cv2.CAP_PROP_AUTO_EXPOSURE,-1)
print vc.get(cv2.CAP_PROP_EXPOSURE)
print vc.get(cv2.CAP_PROP_AUTO_EXPOSURE)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
    #frame=np.rot90(frame_o, 2)
else:
    rval = False
ii=100
while rval:
    dst=cv2.resize(frame, (1280,960), cv2.INTER_LINEAR)
    cv2.imshow("preview", dst)
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