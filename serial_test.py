# -*- coding: utf-8 -*-
"""
Serial_test.py

Sends commands to Arduino Uno via serial port to control a drone
using the nRF24L01 wireless boards.

The arrow keys control elevator and aileron (forward/reverse and left/right)
and the w,s keys control throttle, and the a,d, keys control the rudder (yaw)

This uses the msvcrt library, so it only works under Windows. 

Created on Sun Feb 21 00:17:38 2016

@author: perrytsao
"""
import serial, time, msvcrt

throttle=1000
aileron=1500
elevator=1500
rudder=1500 // yaw, rotates the drone

tg=10
ag=50
eg=50
rg=50
try:
    arduino=serial.Serial('COM3', 115200, timeout=.01)
    time.sleep(1) #give the connection a second to settle
    #arduino.write("1500, 1500, 1500, 1500\n")
    while True:
        
        data = arduino.readline()
        if data:
            #String responses from Arduino Uno are prefaced with [AU]
            print "[AU]: "+data 
            
        if msvcrt.kbhit():
            key = ord(msvcrt.getch())
            if key == 27: #ESC
                print "[PC]: ESC exiting"
                break
            elif key == 13: #Enter
                #select()
                print "[PC]: Enter"
            elif key == 119: #w
                throttle+=tg
            elif key == 97: #a
                rudder-=rg         
            elif key == 115: #s
                throttle-=tg
            elif key == 100: #d
                rudder+=rg
            elif key == 224: #Special keys (arrows, f keys, ins, del, etc.)
                key = ord(msvcrt.getch())
                if key == 80: #Down arrow
                    elevator-=eg
                elif key == 72: #Up arrow
                    elevator+=eg
                elif key == 77: #right arroww
                    aileron+=ag
                elif key == 75: #left arrow
                    aileron-=ag               
            
            command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)
            # string commands to the Arduino are prefaced with  [PC]           
            print "[PC]: "+command 
            arduino.write(command+"\n")

finally:
    # close the connection
    arduino.close()
    # re-open the serial port which will also reset the Arduino Uno and
    # this forces the quadcopter to power off when the radio loses conection. 
    arduino=serial.Serial('COM3', 115200, timeout=.01)
    arduino.close()
    # close it again so it can be reopened the next time it is run.  