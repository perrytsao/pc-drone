# -*- coding: utf-8 -*-
"""
Created on Mon Feb 08 23:00:39 2016

@author: perrytsao
"""
import cv2
import numpy as np
import pickle
#import itertools

import serial# time# msvcrt
import time
import timeit
from datetime import datetime
timestamp="{:%Y_%m_%d_%H_%M}".format(datetime.now())
 
import control_params as cp
import blob_detect as bd



###############################################
cv2.namedWindow("preview")
vc = cv2.VideoCapture(1)

fname="drone_track_640_480_USBFHD01M"
width=640
height=480
fps=30
wait_time=1
vc.set(cv2.CAP_PROP_FRAME_WIDTH,width)
vc.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
vc.set(cv2.CAP_PROP_FPS,fps) 

fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter(timestamp+'_video.avi',fourcc, 20.0, (width,height),1)

throttle=1000
aileron=1500 # moves left/right
elevator=1500 #moves front back
rudder=1500 # yaw, rotates the drone

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
dz_old=0
dx_old=0
dy_old=0

e_dz=0; e_dx=0; e_dy=0
e_iz=0; e_ix=0; e_iy=0
e_d2z=0; e_d2x=0; e_d2y=0

clamp=lambda n, minn, maxn: (max(min(maxn, n), minn))

THROTTLE_MID=cp.THROTTLE_MID
ELEVATOR_MID=cp.ELEVATOR_MID
AILERON_MID=cp.AILERON_MID

speeds=""

zpos_target=65
xpos_target=350
ypos_target=250

font = cv2.FONT_HERSHEY_SIMPLEX
tic=timeit.default_timer()
toc=0
flighttic=timeit.default_timer()
flighttoc=timeit.default_timer()
flightnum=0

recording_data=0
try: 
    arduino=serial.Serial('COM3', 115200, timeout=.001)
    time.sleep(1) #give the connection a second to settle    
    
    if vc.isOpened(): # try to get the first frame
        rval, frame_o = vc.read()
        #frame_undistort=bd.undistort_crop(np.rot90(frame_o, 2))
        frame_undistort=bd.undistort_crop(frame_o)   
        frame, zpos, xypos=bd.add_blobs(frame_undistort)
        #frame, zpos, xypos=bd.add_blobs(frame_o)
    else:
        rval = False
    ii=100
    while rval:
        toc_old=toc        
        toc=timeit.default_timer()
        # prints out time since the last frame was read
        print("deltaT: %0.4f  fps: %0.1f" % (toc - toc_old, 1/(toc-toc_old)))
            
        frame_undistort=bd.undistort_crop(frame_o)
        frame, zpos, xypos=bd.add_blobs(frame_undistort)
        
        #Calculate speed from image
        try:
            dx_old=dx
            dy_old=dy            
            dz_old=dz
            
            dz=(zpos-oldz)*(1-cp.Fz)+cp.Fz*dz
            dx=(xypos[0]-oldx)*(1-cp.Fx)+(cp.Fx)*dx
            dy=(xypos[1]-oldy)*(1-cp.Fy)+(cp.Fy)*dy
            
            oldz=zpos
            oldx=xypos[0]
            oldy=xypos[1]        
        except:
            print "no speed"
        
        if start_flying:
            
            # start recording to video when flying
            #frame_pad=cv2.copyMakeBorder(frame,91,0,75,00,cv2.BORDER_CONSTANT,value=[255,0,0])
            #out.write(frame_pad)
            try: 
                print "Zpos: %i Xpos: %i Ypos: %i" % (zpos, xypos[0], xypos[1])

                e_dz_old=e_dz
                e_dz=dz-zspeed    
                e_iz+=e_dz
                e_iz=clamp(e_iz, -200, 200)
                #e_d2z=e_dz-e_dz_old
                e_d2z=dz-dz_old #ignore command for derivative term
                throttle= cp.Kz*(e_dz*cp.Kpz+cp.Kiz*e_iz+cp.Kdz*e_d2z)+THROTTLE_MID
  
                e_dx_old=e_dx   
                e_dx=dx-xspeed    
                e_ix+=e_dx
                e_ix=clamp(e_ix, -200, 200)
                #e_d2x=e_dx-e_dx_old
                e_d2x=dx-dx_old
                aileron = cp.Kx*(e_dx*cp.Kpx+cp.Kix*e_ix+cp.Kdx*e_d2x)+AILERON_MID   
                
                e_dy_old=e_dy
                e_dy=dy-yspeed    
                e_iy+=e_dy
                e_iy=clamp(e_iy, -200, 200)
                #e_d2y=e_dy-e_dy_old
                e_d2y=dy-dy_old
                elevator= cp.Ky*(e_dy*cp.Kpy+cp.Kiy*e_iy+cp.Kdy*e_d2y)+ELEVATOR_MID
                
                if zpos > 0:
                    print "highalt"
                    aileron=clamp(aileron, 1400, 1600)
                    elevator=clamp(elevator, 1400, 1600)
                else: 
                    print "lowalt" 
                    aileron=clamp(aileron, 1499, 1501)
                    elevator=clamp(elevator, 1499, 1501)
                # set target speeds                
                
                zpeed_old=zspeed
                zspeed=-1*(zpos-zpos_target)*cp.Ksz
                zspeed=clamp(zspeed, -0.1, 0.5)
                
                xspeed=-1*(xypos[0]-xpos_target)*cp.Ksx
                xspeed=clamp(xspeed, -0.02, 0.02)
                
                yspeed=-1*(xypos[1]-ypos_target)*cp.Ksy
                yspeed=clamp(yspeed, -0.02, 0.02)
#                if zpos > zpos_target:
#                    zspeed=-.1
#                else:
#                    zspeed=+.5
#                if xypos[0] > xpos_target:
#                    xspeed=-0.2 # move left
#                else:
#                    xspeed=0.2 # move right
#                if xypos[1] > ypos_target:
#                    yspeed=-0.2 # move up
#                else:
#                    yspeed=+0.2 # move down
                     
                no_position_cnt=0    
            except Exception as e:
                print (e)
                no_position_cnt+=1
                print "STOPPED. no position or error. "
                if no_position_cnt>15:
                    throttle=1000
                    start_flying=0  
        
        ## Serial comms - write to Arduino
        throttle=clamp(throttle, 1000, 2000)
        command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)
        print "[PC]: "+command
        arduino.write(command+"\n")
                
        ## Serial comms - read back from Arduino
        data = arduino.readline()
        while data:
            print "[AU]: "+data.rstrip("\n") #strip out the new lines for now
            # (better to do .read() in the long run for this reason    
            data=arduino.readline()
        
        ## Monitor keyboard
        speeds= "dz:  %+5.2f dx:  %+5.2f  dy: %+5.2f" % (dz, dx, dy)     
        targets="tsz: %+5.2f tsx: %+5.2f tsy: %+5.2f" % (zspeed, xspeed, yspeed)  
        gains="Kpz: %+5.2f Kiz: %+5.2f Kdz: %+5.2f" % (cp.Kpz, cp.Kiz, cp.Kdz)  
        errors_z="e_dz: %+5.2f e_iz: %+5.2f e_d2z: %+5.2f" % (e_dz, e_iz, e_d2z) 
        flighttoc=timeit.default_timer()
        cv2.putText(frame, command,(10,50), font, .8,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame, speeds,(10,75), font, .8,(255,255,255),2,cv2.LINE_AA)        
        cv2.putText(frame, targets,(10,100), font, .8,(255,255,255),2,cv2.LINE_AA) 
        cv2.putText(frame, gains,(10,125), font, .8,(255,255,255),2,cv2.LINE_AA)         
        cv2.putText(frame, errors_z,(10,150), font, .8,(255,255,255),2,cv2.LINE_AA)         
        cv2.putText(frame, 'Flt#: {0} Time:{1:0.3f}'.format(flightnum,flighttoc-flighttic),(10,175), font, .8,(255,255,255),2,cv2.LINE_AA)
        #dst=cv2.resize(frame, (1280,960), cv2.INTER_NEAREST)
        cv2.imshow("preview", frame)
        key = cv2.waitKey(wait_time)
        if start_flying:
            # start recording to video when flying
            frame_pad=cv2.copyMakeBorder(frame,91,0,75,00,cv2.BORDER_CONSTANT,value=[255,0,0])
            out.write(frame_pad)
            if xypos is None:
                xypos=np.zeros(2)
                zpos=0
            flightdata=np.vstack((flightdata, np.array([flighttoc-flighttic, 
                        xypos[0], xypos[1], zpos,
                        dx, dy, dz,
                        e_dx, e_ix, e_d2x,
                        e_dy, e_iy, e_d2y,
                        e_dz, e_iz, e_d2z,
                        xspeed, yspeed, zspeed,
                        throttle, aileron, elevator, rudder])))
        elif recording_data:
            np.save(timestamp+'_flt'+str(flightnum)+'_'+'flightdata.npy', flightdata)
            np.save(timestamp+'_flt'+str(flightnum)+'_'+'controldata.npy', controldata)
            with open(timestamp+'_flt'+str(flightnum)+'_'+'controlvarnames.npy', 'wb') as f:
                pickle.dump(controlvarnames, f)
            recording_data=0
            
            
        if key == 27: # exit on ESC
            break
        elif key == 32: # space - take a snapshot and save it
            cv2.imwrite(fname+str(ii)+".jpg", frame)
            ii+=1
        elif key == 119: #w
            throttle=THROTTLE_MID
            aileron=AILERON_MID # turns left
            elevator=ELEVATOR_MID
            e_ix = 0; e_iy = 0; e_iz = 0
            rudder=1500 # yaw, rotates the drone
            start_flying=1
            recording_data=1
            flightdata=np.zeros(23)
            flighttic=timeit.default_timer()
            flighttoc=0
            flightnum+=1
            '''np.array([xpos, ypos, zpos,
            dx, dy, dz,
            e_dx, e_ix, e_d2x,
            e_dy, e_iy, e_d2y,
            e_dz, e_iz, e_d2z,
            xspeed, yspeed, zspeed,
            throttle, aileron, elevator, rudder])'''
            reload(cp)
            # this lists out all the variables in module cp
            # and records their values. 
            controlvarnames=[item for item in dir(cp) if not item.startswith("__")]
            controldata=[eval('cp.'+item) for item in controlvarnames]
            
            
            print "START FLYING"
        elif key == 115: #s
            throttle=1000
            start_flying=0
        elif key == 114: #r - reset the serial port so Arduino will bind to another CX-10
            arduino.close()
            arduino=serial.Serial('COM3', 115200, timeout=.001)
    
        # print out the time needed to execute everything except the image reload
        toc2=timeit.default_timer()
        print("deltaT_execute: %0.4f" % (toc2 - toc))
        
        # read next frame
        rval, frame_o = vc.read()
        
            
finally:
    # close the connection
    arduino.close()
    # re-open the serial port which will w for Arduino Uno to do a reset
    # this forces the quadcopter to power off motors.  Will need to power
    # cycle the drone to reconnect
    arduino=serial.Serial('COM3', 115200, timeout=.001)
    arduino.close()
    # close it again so it can be reopened the next time it is run.      
    
    vc.release()
    cv2.destroyWindow("preview")
    out.release()
