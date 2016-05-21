# -*- coding: utf-8 -*-
"""
Created on Mon Feb 08 23:00:39 2016

@author: perrytsao
"""

import cv2

cv2.namedWindow("preview")
vc = cv2.VideoCapture(1)

width=800
height=600
wait_time=33
fps=int(1/wait_time/.01)
vc.set(3,width)
vc.set(4,height)


#out = cv2.VideoWriter('output.avi', -1, 50.0, (1080,720))
#fourcc = cv2.cv.CV_FOURCC(*'DIVX')
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter('output9.avi',fourcc, 33.0, (width,height),1)
if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

while rval:
    cv2.imshow("preview", frame)
    rval, frame = vc.read()
    out.write(frame)
    key = cv2.waitKey(wait_time)
    if key == 27: # exit on ESC
        break
vc.release()
cv2.destroyWindow("preview")
out.release()