#!/usr/bin/env python3

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
from numpy import linalg as LA


filename = './'

class AddVelProfile():

    def __init__(self):
        print('------- Modifying Waypoints ------')
        self.modify_waypoint()
        

    def modify_waypoint(self):
        filename = '/home/moody/f1tenth_ws/waypoints.csv'
        waypoints = np.genfromtxt(filename, delimiter=',')
        num_waypoints = np.shape(waypoints)[0]

        #create cubic spline
        t = np.linspace(0, num_waypoints-1, num_waypoints)
        x = waypoints[:,:2]
        
        #filter a bit for drivatives
        sigma = 3
        x_filt = gaussian_filter(x[:,0], sigma=sigma, mode='reflect')
        y_filt = gaussian_filter(x[:,1], sigma=sigma, mode='reflect')

        cs_x = CubicSpline(t, x_filt)
        cs_y = CubicSpline(t, y_filt)
        plt.plot(cs_x(t), cs_y(t))
        plt.waitforbuttonpress(3)

        # Calculate derivatives
        dx = cs_x.derivative(1)
        ddx = cs_x.derivative(2) 
        dy = cs_y.derivative(1)
        ddy = cs_y.derivative(2)

        curvature = np.abs((dx(t) * ddy(t) - dy(t) * ddx(t)) / np.power(dx(t)**2 + dy(t)**2, 3/2))
        curvature[:10] = 0
        curvature[-10:] = 0

        # normalize 
        curvature = curvature / np.max(curvature)
        plt.plot(t, curvature, label='curvature')

        v_max = 2
        v_min = 0.5
        velocities = v_min + (v_max-v_min)*(1-curvature)
        plt.plot(t, velocities, label='velocity profile')
        plt.legend()
        plt.waitforbuttonpress()

        new_waypoints = np.hstack((waypoints,velocities.reshape(num_waypoints,1)))
        np.savetxt(filename.replace('.csv', '_with_velocities2.csv'), new_waypoints, delimiter=',', fmt='%f')
        print('------- Waypoint Modified! ------')
        print('Goodbye')



if __name__ == '__main__':
    AddVelProfile()
    
