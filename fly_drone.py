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
# drone parameters
mass=.014 # 14g for drone and cage and the markers
          # 50px in x,y directions = 7cm
          # Distance between blobs = 7cm
          # 49.1 px (distance between blobs) => 23.25" height
          # 74 px => 15.5" height
# z axis flight sequence calculations
# maxrate= 1 cm/s
# 20 samples / s
# 25px in z axis = 8 in = 20cm
# 1.25px / cm
# 1.25px / s
# 0.0625px / sample

###############################################

def flight_sequence(seqname, xseq_list, yseq_list, zseq_list, tseq_list):
    # This function takes sequence lists and returns sequence lists.
    # Internally it uses numpy arrays.
    # 
    # THe sequence lists must have some length so that the starting position
    # is known. Empty lists are not allowed.
    xseq=np.array(xseq_list)        
    yseq=np.array(yseq_list)            
    zseq=np.array(zseq_list)            
    tseq=np.array(tseq_list)        
    
    seqrate = 2    
    if seqname == 'land':
        zpoints=np.abs(np.round((zseq[-1]-45)/seqrate))
        zseq=np.concatenate((zseq, np.linspace(zseq[-1], 30, zpoints)))
        xseq=np.concatenate((xseq, np.ones(zpoints)*xseq[-1]))
        yseq=np.concatenate((yseq, np.ones(zpoints)*yseq[-1]))
        tseq=np.concatenate((tseq, np.ones(zpoints)*tseq[-1]))
    elif seqname == 'takeoff':
        zpoints=np.abs(np.round((zseq[-1]-65)/seqrate))
        zseq=np.concatenate((zseq, np.linspace(zseq[-1], 65, zpoints)))
        xseq=np.concatenate((xseq, np.ones(zpoints)*xseq[-1]))
        yseq=np.concatenate((yseq, np.ones(zpoints)*yseq[-1]))
        tseq=np.concatenate((tseq, np.ones(zpoints)*tseq[-1]))
    elif seqname == 'box': # goes in a 10cm box pattern 
        pts=np.abs(np.round((75)/seqrate))        
        fwd=np.linspace(0, 75, pts)
        
        xseq=np.concatenate((xseq, fwd+xseq[-1]))
        xseq=np.concatenate((xseq, np.ones(pts)*xseq[-1]))
        xseq=np.concatenate((xseq, (-1*fwd)+xseq[-1]))
        xseq=np.concatenate((xseq, np.ones(pts)*xseq[-1]))
        
        yseq=np.concatenate((yseq, np.ones(pts)*yseq[-1]))        
        yseq=np.concatenate((yseq, (-1*fwd)+yseq[-1]))
        yseq=np.concatenate((yseq, np.ones(pts)*yseq[-1]))
        yseq=np.concatenate((yseq, (fwd)+yseq[-1]))
        
        zseq=np.concatenate((zseq, np.ones(4*pts)*zseq[-1]))
        tseq=np.concatenate((tseq, np.ones(pts)*tseq[-1]))
    elif seqname == 'up':
        zpoints=np.abs(np.round(12/seqrate))
        zseq=np.concatenate((zseq, np.linspace(zseq[-1], zseq[-1]+12, zpoints)))
        xseq=np.concatenate((xseq, np.ones(zpoints)*xseq[-1]))
        yseq=np.concatenate((yseq, np.ones(zpoints)*yseq[-1]))
        tseq=np.concatenate((tseq, np.ones(zpoints)*tseq[-1]))
    elif seqname == 'down':
        zpoints=np.abs(np.round(12/seqrate))
        zseq=np.concatenate((zseq, np.linspace(zseq[-1], zseq[-1]-12, zpoints)))
        xseq=np.concatenate((xseq, np.ones(zpoints)*xseq[-1]))
        yseq=np.concatenate((yseq, np.ones(zpoints)*yseq[-1]))   
        tseq=np.concatenate((tseq, np.ones(zpoints)*tseq[-1]))
    elif seqname == 'left_spot':    
        xpoints=np.abs(np.round((xseq[-1]-200)/1))
        xseq=np.concatenate((xseq, np.linspace(xseq[-1], 200, xpoints)))
        yseq=np.concatenate((yseq, np.ones(xpoints)*yseq[-1]))
        zseq=np.concatenate((zseq, np.ones(xpoints)*zseq[-1]))
        tseq=np.concatenate((tseq, np.ones(xpoints)*tseq[-1]))
    elif seqname == 'right_spot':    
        xpoints=np.abs(np.round((xseq[-1]-400)/1))
        xseq=np.concatenate((xseq, np.linspace(xseq[-1], 400, xpoints)))
        yseq=np.concatenate((yseq, np.ones(xpoints)*yseq[-1]))
        zseq=np.concatenate((zseq, np.ones(xpoints)*zseq[-1]))
        tseq=np.concatenate((tseq, np.ones(xpoints)*tseq[-1]))
    elif seqname == 'hover':    
        xpoints=300 # 15s of hovering in one spot
        xseq=np.concatenate((xseq, np.ones(xpoints)*xseq[-1]))
        yseq=np.concatenate((yseq, np.ones(xpoints)*yseq[-1]))
        zseq=np.concatenate((zseq, np.ones(xpoints)*zseq[-1]))
        tseq=np.concatenate((tseq, np.ones(xpoints)*tseq[-1]))
    elif seqname == 'rot90_left':     # this code does not take care of rotating past 180 degrees
        xpoints=150 
        theta_endpoint=tseq[-1]+np.pi/2
        if (theta_endpoint > np.pi):
            theta_endpoint-=2*np.pi
        elif (e_dt < (-1*np.pi)):
            theta_endpoint+=2*np.pi        
        xseq=np.concatenate((xseq, np.ones(xpoints)*xseq[-1]))
        yseq=np.concatenate((yseq, np.ones(xpoints)*yseq[-1]))
        zseq=np.concatenate((zseq, np.ones(xpoints)*zseq[-1]))
        
        tseq=np.concatenate((tseq, np.linspace(tseq[-1], theta_endpoint, xpoints)))
    elif seqname == 'rot90_right':     # this code does not take care of rotating past 180 degrees
        xpoints=150 
        theta_endpoint=tseq[-1]-np.pi/2
        if (theta_endpoint > np.pi):
            theta_endpoint-=2*np.pi
        elif (e_dt < (-1*np.pi)):
            theta_endpoint+=2*np.pi        
        xseq=np.concatenate((xseq, np.ones(xpoints)*xseq[-1]))
        yseq=np.concatenate((yseq, np.ones(xpoints)*yseq[-1]))
        zseq=np.concatenate((zseq, np.ones(xpoints)*zseq[-1]))
        
        tseq=np.concatenate((tseq, np.linspace(tseq[-1], theta_endpoint, xpoints)))    
    return list(xseq), list(yseq), list(zseq), list(tseq)

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
out = cv2.VideoWriter('flight_data\\'+timestamp+'_video.avi',fourcc, 20.0, (width,height),1)

throttle=1000
aileron=1500 # moves left/right
elevator=1500 #moves front back
rudder=1500 # yaw, rotates the drone

zpos=50
xypos=(350, 250)
theta=0

command=""
start_flying=0
no_position_cnt=0

dz=0
dx=0
dy=0
xspeed=0
yspeed=0
zspeed=0
dz_old=0
dx_old=0
dy_old=0

e_dz=0; e_dx=0; e_dy=0; e_dt=0
e_iz=0; e_ix=0; e_iy=0; e_it=0
e_d2z=0; e_d2x=0; e_d2y=0; e_d2t=0

clamp=lambda n, minn, maxn: (max(min(maxn, n), minn))

THROTTLE_MID=cp.THROTTLE_MID
ELEVATOR_MID=cp.ELEVATOR_MID
AILERON_MID=cp.AILERON_MID
RUDDER_MID=cp.RUDDER_MID

speeds=""

xpos_target=300
ypos_target=200
zpos_target=65
theta_target=0#45.0/180.0*np.pi

xpos_target_seq=[xpos_target]
ypos_target_seq=[ypos_target]
zpos_target_seq=[zpos_target]
theta_target_seq=[theta_target]

font = cv2.FONT_HERSHEY_SIMPLEX
tic=timeit.default_timer()
toc=0
flighttic=timeit.default_timer()
flighttoc=timeit.default_timer()
flightnum=0

# Flight modes
NORMAL_FM=0
LANDING_FM=1
PROGRAM_SEQ_FM=2

flt_mode=NORMAL_FM

recording_data=0
try: 
    arduino=serial.Serial('COM4', 115200, timeout=.001)
    time.sleep(1) #give the connection a second to settle    
    
    if vc.isOpened(): # try to get the first frame
        rval, frame_o = vc.read()
        #frame_undistort=bd.undistort_crop(np.rot90(frame_o, 2))
        frame_undistort=bd.undistort_crop(frame_o)   
        frame, zpos, xypos, theta=bd.add_blobs(frame_undistort)
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
        toc2=timeit.default_timer()
        print("deltaT_execute_undistort: %0.4f" % (toc2 - toc))

        frame, zpos, xypos, theta=bd.add_blobs(frame_undistort)
                
        toc2=timeit.default_timer()
        print("deltaT_execute_blob_detect: %0.4f" % (toc2 - toc))
        
        if start_flying:
            try: 
                if flt_mode <> LANDING_FM:
                    print "Zpos: %i Xpos: %i Ypos: %i" % (zpos, xypos[0], xypos[1])
    
                    e_dz_old=e_dz                
                    e_dz=zpos-zpos_target
                    e_iz+=e_dz
                    e_iz=clamp(e_iz, -10000, 10000)
                    e_d2z=e_dz-e_dz_old
                    throttle= cp.Kz*(e_dz*cp.Kpz+cp.Kiz*e_iz+cp.Kdz*e_d2z)+THROTTLE_MID    
      
                    e_dx_old=e_dx   
                    e_dx=xypos[0]-xpos_target
                    e_ix+=e_dx
                    e_ix=clamp(e_ix, -200000, 200000)
                    e_d2x=e_dx-e_dx_old
                    #aileron = cp.Kx*(e_dx*cp.Kpx+cp.Kix*e_ix+cp.Kdx*e_d2x)+AILERON_MID   
                    xcommand= cp.Kx*(e_dx*cp.Kpx+cp.Kix*e_ix+cp.Kdx*e_d2x)
                    
                    
                    e_dy_old=e_dy
                    e_dy=xypos[1]-ypos_target    
                    e_iy+=e_dy
                    e_iy=clamp(e_iy, -200000, 200000)
                    e_d2y=e_dy-e_dy_old
                    #elevator= cp.Ky*(e_dy*cp.Kpy+cp.Kiy*e_iy+cp.Kdy*e_d2y)+ELEVATOR_MID
                    ycommand=cp.Ky*(e_dy*cp.Kpy+cp.Kiy*e_iy+cp.Kdy*e_d2y)
                    
                    # commands are calculated in camera reference frame
                    aileron=xcommand*np.cos(theta)+ycommand*np.sin(theta) + AILERON_MID
                    elevator=-1*xcommand*np.sin(theta)+ycommand*np.cos(theta) + ELEVATOR_MID
                    
                    e_dt_old=e_dt
                    e_dt=theta-theta_target    
                    # angle error should always be less than 180degrees (pi radians)
                    if (e_dt > np.pi):
                        e_dt-=2*np.pi
                    elif (e_dt < (-1*np.pi)):
                        e_dt+=2*np.pi
                                           
                    e_it+=e_dt
                    e_it=clamp(e_it, -200000, 200000)
                    e_d2t=e_dt-e_dt_old
                    rudder= cp.Kt*(e_dt*cp.Kpt+cp.Kit*e_it+cp.Kdt*e_d2t)+RUDDER_MID                    
                    
                    if zpos > 0:
                        print "highalt"
                        aileron=clamp(aileron, 1000, 2000)
                        elevator=clamp(elevator, 1000, 2000)
                    else: 
                        print "lowalt" 
                        aileron=clamp(aileron, 1400, 1600)
                        elevator=clamp(elevator, 1400, 1600)
                    no_position_cnt=0
                else: # landing mode
                    throttle=throttle-20
                
                
                
            except Exception as e:
                print (e)
                no_position_cnt+=1
                print "STOPPED. no position or error. "
                if no_position_cnt>15:
                    throttle=1000
                    start_flying=0  
        
        ## Serial comms - write to Arduino
        throttle=clamp(throttle, 1000, 2000)
        rudder=clamp(rudder, 1000, 2000)
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
        #cv2.putText(frame, speeds,(10,75), font, .8,(255,255,255),2,cv2.LINE_AA)        
        #cv2.putText(frame, targets,(10,100), font, .8,(255,255,255),2,cv2.LINE_AA) 
        #cv2.putText(frame, gains,(10,125), font, .8,(255,255,255),2,cv2.LINE_AA)         
        #cv2.putText(frame, errors_z,(10,150), font, .8,(255,255,255),2,cv2.LINE_AA)         
        cv2.putText(frame, 'Flt#: {0} Time:{1:0.3f}'.format(flightnum,flighttoc-flighttic),(10,75), font, .8,(255,255,255),2,cv2.LINE_AA)
        cv2.rectangle(frame, (int(xpos_target)-5, int(ypos_target)-5), (int(xpos_target)+5, int(ypos_target)+5), (255,0,0), thickness=1, lineType=8, shift=0)
        #dst=cv2.resize(frame, (1280,960), cv2.INTER_NEAREST)
        cv2.imshow("preview", frame)
        toc2=timeit.default_timer()
        print("deltaT_execute_imshow: %0.4f" % (toc2 - toc))        
        
        key = cv2.waitKey(wait_time)
        toc2=timeit.default_timer()
        print("deltaT_execute_waitkey: %0.4f" % (toc2 - toc))
        #key = ord('0')
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
            if len(xpos_target_seq) > 1:
                xpos_target=xpos_target_seq.pop(0)
                ypos_target=ypos_target_seq.pop(0)
                zpos_target=zpos_target_seq.pop(0)
                theta_target=theta_target_seq.pop(0)
                print 'seq len %i' % len(xpos_target_seq)
            elif flt_mode == PROGRAM_SEQ_FM:
                flt_mode = LANDING_FM
                
        elif recording_data:
            np.save('flight_data\\'+timestamp+'_flt'+str(flightnum)+'_'+'flightdata.npy', flightdata)
            np.save('flight_data\\'+timestamp+'_flt'+str(flightnum)+'_'+'controldata.npy', controldata)
            with open('flight_data\\'+timestamp+'_flt'+str(flightnum)+'_'+'controlvarnames.npy', 'wb') as f:
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

            reload(cp)
            # this lists out all the variables in module cp
            # and records their values. 
            controlvarnames=[item for item in dir(cp) if not item.startswith("__")]
            controldata=[eval('cp.'+item) for item in controlvarnames]
            flt_mode=NORMAL_FM           
            print "START FLYING"
        elif key == ord('e'): 
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

            reload(cp)
            # this lists out all the variables in module cp
            # and records their values. 
            controlvarnames=[item for item in dir(cp) if not item.startswith("__")]
            controldata=[eval('cp.'+item) for item in controlvarnames]    
                        
            xpos_target_seq=[xpos_target]
            ypos_target_seq=[ypos_target]
            zpos_target_seq=[zpos_target] 
            theta_target_seq=[theta_target]
            xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq= flight_sequence(
                'hover', xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq)              
            
            xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq= flight_sequence(
                'right_spot', xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq)              
            
            xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq= flight_sequence(
                'left_spot', xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq)              
                        
            flt_mode=PROGRAM_SEQ_FM
            
            print "START FLYING"            
            
        elif key == 115: #s
            #throttle=1000
            #start_flying=0
            flt_mode = LANDING_FM
        elif key == 114: #r - reset the serial port so Arduino will bind to another CX-10
            arduino.close()
            arduino=serial.Serial('COM4', 115200, timeout=.001)
        elif key == ord('1'):
            xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq= flight_sequence(
                'takeoff', xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq)
        elif key == ord('2'):
            xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq= flight_sequence(
                'land', xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq)
        elif key == ord('3'):
            xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq= flight_sequence(
                'box', xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq)
        elif key == ord('4'):
            xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq= flight_sequence(
                'left_spot', xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq)   
        elif key == ord('5'):
            xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq= flight_sequence(
                'right_spot', xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq)                
        elif key == ord('6'):
            xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq= flight_sequence(
                'rot90_left', xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq)                                
        elif key == ord('7'):
            xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq= flight_sequence(
                'rot90_right', xpos_target_seq, ypos_target_seq, zpos_target_seq, theta_target_seq)                                
        # print out the time needed to execute everything except the image reload
        toc2=timeit.default_timer()
        print("deltaT_execute_other: %0.4f" % (toc2 - toc))
        
        # read next frame
        rval, frame_o = vc.read()
        toc2=timeit.default_timer()
        print("deltaT_execute_nextframe: %0.4f" % (toc2 - toc))
        
finally:
    # close the connection
    arduino.close()
    # re-open the serial port which will w for Arduino Uno to do a reset
    # this forces the quadcopter to power off motors.  Will need to power
    # cycle the drone to reconnect
    arduino=serial.Serial('COM4', 115200, timeout=.001)
    arduino.close()
    # close it again so it can be reopened the next time it is run.      
    vc.release()
    cv2.destroyWindow("preview")
    out.release()