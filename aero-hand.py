# Author: Rajath Bhargav
# See ReadMe for info

import matplotlib.pyplot as plt
import numpy as np
import math

t = np.linspace(0,10,100) # time vector (simulation length)

rho = 1.168 # air density
g = 9.806 # Y'all know this
ho = 1.8 # launch height
hw = 10 # headwind in km/hr
vo = 50 # Forward speed in m/s at which prop generates zero thrust
hcrit = 0.2 # critical altitude for a safe takeoff
f = 0.5 # throw force in kg
Cl = 1.2 # overall wing lift coefficient (ANSYS/xflr5)
CdAd = 0.1 # overall drag coefficient and frontal area product of plane (ANSYS/xflr5)
w = 2*(0.1375+0.33+0.33) # wingspan from FEA, weight and other constraints
theta = 10 # Induced dihedral angle
chord = 0.19 # from maximum thickness of airfoil that can be placed in container
M = 1 # plane mass

Al = w*chord*math.cos(theta * math.pi / 180)
u = (hw/3.6) + math.sqrt(2*g*f*0.4/M)  # assuming 0.4m hand travel and neglecting drag
l = 0.5*rho*Cl*Al
b = 0.5*rho*CdAd
T = (b*vo*u**2)/((vo - u)*g) + 0.01 # thrust force in kg. Minimum T is obtained by the condition that arg_atanh < 1 (see below)

while T <= 20:
    F = T*g
    T = T + 0.01

    # solution to differential equations:

    #v = u+(np.sqrt(F/b)*np.tanh(t*np.sqrt(b*F)/M))

    term1 = math.sqrt(4*b*F*vo**2 + F**2)

    arg_atanh = (F + 2*b*u*vo)/(term1) # this has to be less than 1 => gives starting point for iterating T

    v = (np.tanh((term1*(t/(M*vo) + (2*math.atanh(arg_atanh))/(term1)))/2)*term1-F)/(2*b*vo)

    #v = vo + np.exp(-(F*t)/(M*vo))*(u - vo) # equation without drag

    L = l*(v**2)
    h = ho+((0.5*(L-(M*g))/M)*(t**2)) # neglecting drag in vertical direction
    
    if min(h) >= hcrit:
        break

vmax = (math.sqrt(F**2 + 4*b*F*vo**2)-F)/(2*b*vo)
vstall = 3.6*(math.sqrt(M*g/l))

print('Total plane mass: ',M,' kg')
print('Wing loading: ',M/Al,' kg/m^2')
print('Stall speed: ',vstall,' km/hr')
print('Static thrust required: ',T,' kg')
print('Top speed: ',vmax*3.6,' km/hr')

plt.subplot(311)
plt.plot(t,v)
plt.plot(t,math.sqrt(M*g/l)*np.ones(len(t)),'g')
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
plt.plot(t,hcrit*np.ones(len(t)),'r')
plt.legend(['Instantaneous altitude','Critical altitude'])
plt.axis([0,5,0,3*ho])
plt.grid(True)
plt.xlabel('sec')
plt.ylabel('m')
plt.title('Altitude at constant full throttle')

plt.show()
