# Author: Rajath Bhargav
# See ReadMe for info

import matplotlib.pyplot as plt
import numpy as np
import math

t_max = 10 # simulation length in seconds
t = np.linspace(0,t_max,1000) # time vector
rho = 1.168 # air density
g = 9.806 # Y'all know this
R = 10 # Runway length in m
hw = 1 # headwind in km/h (indicate tailwind in negative)
vo = 50 # Forward speed in m/s at which prop generates zero thrust at max RPM
Cl = 1.2 # overall wing lift coefficient (ANSYS/xflr5)
CdAd = 0.1 # overall drag coefficient and frontal area product of plane (ANSYS/xflr5)
w = 2*(0.1375+0.33+0.33) # wingspan from FEA, weight and other constraints
theta = 10 # Induced dihedral angle
chord = 0.19 # from maximum thickness of airfoil that can be placed in container
M = 1 # plane mass

hw = hw/3.6
Al = w*chord*math.cos(theta * math.pi / 180)
l = 0.5*rho*Cl*Al
b = 0.5*rho*CdAd
T = (b*vo*hw**2)/(g*(vo-hw)) + 0.01 # static thrust force in kg. Minimum T is obtained by the condition that arg_atanh < 1 (see below)
v_stall = math.sqrt(M*g/l)

while T < 5:
    T = T + 0.01 # increment thrust
    
    F = T*g

    # solution to differential equations:

    #v = u+(np.sqrt(F/b)*np.tanh(t*np.sqrt(b*F)/M))

    term1 = math.sqrt(4*b*F*vo**2 + F**2)

    arg_atanh = (F + 2*b*hw*vo)/(term1) # this has to be less than 1 => gives starting point for iterating T

    v = (np.tanh((term1*(t/(M*vo) + (2*math.atanh(arg_atanh))/(term1)))/2)*term1-F)/(2*b*vo) # v(t)

    #v = vo + np.exp(-(F*t)/(M*vo))*(hw - vo) # equation without drag

    L = l*(v**2) # L(t)
    h = (0.5*(L-(M*g))/M)*(t**2) # h(t) neglecting drag in vertical direction
    h[h<0] = 0 # no negative height in case of runway
    
    take_off = 0
    for vel in v:
        if vel > v_stall: # if velocity > stall speed
            break         # aircraft has took off
        take_off = take_off+1
    
    if take_off < len(v)-1:
        r = np.trapz(v[0:take_off+1],t[0:take_off+1]) # total runway distance covered till the point of take off (trapezoidal integration)
        if r < 0.9*R: # distance taken less than runway length - safety margin
            break  # we have enough thrust

v_max = (math.sqrt(F**2 + 4*b*F*vo**2)-F)/(2*b*vo)

print('Total plane mass: ',M,' kg')
print('Wing loading: ',M/Al,' kg/m^2')
print('Stall speed: ',v_stall*3.6,' km/hr')
print('Static thrust required: ',T,' kg')
print('Top speed: ',v_max*3.6,' km/hr')

plt.subplot(311)
plt.plot(t,v)
plt.plot(t,v_stall*np.ones(len(t)),'g')
plt.grid(True)
plt.xlabel('sec')
plt.ylabel('m/s')
plt.title('Velocity at constant full throttle')
plt.legend(['Instantaneous velocity','Stall speed'])

plt.subplot(312)
plt.plot(t,L/g)
plt.plot(t,M*np.ones(len(t)),'g')
plt.grid(True)
plt.xlabel('sec')
plt.ylabel('kg')
plt.title('Lift at constant full throttle')
plt.legend(['lift','Plane mass'])

plt.subplot(313)
plt.plot(t,h)
plt.legend(['Instantaneous altitude'])
plt.grid(True)
plt.xlabel('sec')
plt.ylabel('m')
plt.title('Altitude v/s time')

plt.show()
