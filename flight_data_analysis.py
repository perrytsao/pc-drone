# -*- coding: utf-8 -*-
"""
Created on Sat Apr 02 20:35:11 2016

@author: perrytsao
"""

import numpy as np
import matplotlib.pyplot as plt
import sys

plt.close('all')

if len(sys.argv)>1:
    fltname='flight_data\\'+sys.argv[1]    
else:
    fltname='flight_data\\'+'2016_04_09_14_51_flt4_'
controldata=np.load(fltname+'controldata.npy')
controlvarnames=np.load(fltname+'controlvarnames.npy')
flightdata=np.load(fltname+'flightdata.npy')

cpdict=dict(zip(controlvarnames, controldata))

flight_data_names=['t', 'x', 'y', 'z',
                        'dx', 'dy', 'dz',
                        'e_dx', 'e_ix', 'e_d2x',
                        'e_dy', 'e_iy', 'e_d2y',
                        'e_dz', 'e_iz', 'e_d2z',
                        'xspeed', 'yspeed', 'zspeed',
                        'throttle', 'aileron', 'elevator', 'rudder']
#index of flight data
fd=flightdata[1:, :]
fi=dict(zip(flight_data_names,range(23) ))

for xx in fi.keys():
   exec(xx+'=fd[:,fi[xx]]')    
   
t=t-t[0]

fig1=plt.figure(1)
plt.clf()
plt.plot(t, x, t, y, t,z)
fig1.canvas.set_window_title('Position (x,y,z)')
plt.show()

fig2=plt.figure(2)
plt.clf()
plt.subplot(211)
plt.plot(t, x, t, y, t,z)
plt.hold(True)
plt.plot([t[0], t[-1]], [350, 350])
plt.plot([t[0], t[-1]], [250, 250])
plt.plot([t[0], t[-1]], [65, 65])
plt.subplot(212)

plt.plot(t, dx, t, dy, t,dz)
plt.hold(True)
# post-calculated velocity (no filtering)
compdx=-1*(x[0:-1]-x[1:])
compdy=-1*(y[0:-1]-y[1:])
compdz=-1*(z[0:-1]-z[1:])
#plt.plot(t[1:], compdx, t[1:], compdy, t[1:],compdz, hold=True)

plt.plot(t, xspeed, t, yspeed, t, zspeed)
fig2.canvas.set_window_title('Velocities (dx, dy, dz)')
plt.show()

fig3=plt.figure(3)
plt.clf()
plt.plot(t,e_dx, t,e_dy, t,e_dz)
fig3.canvas.set_window_title('Velocity Error (e_dx, e_dy, e_dz)')
plt.show()
    
fig4=plt.figure(4)
plt.clf()
plt.plot(t, aileron, t,elevator, t, throttle)
fig4.canvas.set_window_title('Aileron-elevator-throttle')
plt.show()

fig5=plt.figure(5)
plt.clf()
plt.subplot(211)
plt.plot(t, xspeed, t, yspeed, t, zspeed)
plt.subplot(212)
plt.plot(t, dx, t, dy, t,dz)
fig5.canvas.set_window_title('Target Velocities (xspeed, yspeed, zspeed)')
plt.show()

fig6=plt.figure(6)
plt.clf()
plt.subplot(311)
#aileron = cp.Kx*(e_dx*cp.Kpx+cp.Kix*e_ix+cp.Kdx*e_d2x)+AILERON_MID  
plt.plot(t, e_dx*cpdict['Kpx'], t, e_ix*cpdict['Kix'], t, e_d2x*cpdict['Kdx'])
plt.subplot(312)
plt.plot(t, e_dy*cpdict['Kpy'], t, e_iy*cpdict['Kiy'], t, e_d2y*cpdict['Kdy'])
plt.subplot(313)
plt.plot(t, e_dz*cpdict['Kpz'], label='P')
plt.plot(t, e_iz*cpdict['Kiz'], label='I')
plt.plot(t, e_d2z*cpdict['Kdz'],label='D')
fig6.canvas.set_window_title('Control ouputs - PID')
plt.legend()
plt.show()

