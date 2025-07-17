import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import matplotlib.animation as animation
import datetime
import matplotlib.dates as mdates
from collections import deque
import numpy as np
import re
import math
#from .lis3mdl import LIS3MDL
from scipy import linalg
import os

class Magnetometer(object):
    
    '''
        To obtain Gravitation Field (raw format):
    1) get the Total Field for your location from here:
       http://www.ngdc.noaa.gov/geomag-web (tab Magnetic Field)
       es. Total Field = 47,241.3 nT | my val :47'789.7
    2) Convert this values to Gauss (1nT = 10E-5G)
       es. Total Field = 47,241.3 nT = 0.47241G
    3) Convert Total Field to Raw value Total Field, which is the
       Raw Gravitation Field we are searching for
       Read your magnetometer datasheet and find your gain value,
       Which should be the same of the collected raw points
       es. on HMC5883L, given +_ 1.3 Ga as Sensor Field Range settings
           Gain (LSB/Gauss) = 1090 
           Raw Total Field = Gain * Total Field
           0.47241 * 1090 = ~515  |
           
        -----------------------------------------------
         gain (LSB/Gauss) values for HMC5883L
            0.88 Ga => 1370 
            1.3 Ga => 1090 
            1.9 Ga => 820
            2.5 Ga => 660 
            4.0 Ga => 440
            4.7 Ga => 390 
            5.6 Ga => 330
            8.1 Ga => 230 
        -----------------------------------------------

     references :
        -  https://teslabs.com/articles/magnetometer-calibration/      
        -  https://www.best-microcontroller-projects.com/hmc5883l.html

    '''
    magField = 0.458327
    gain = 6842
    MField = magField*gain
    print(MField)

    def __init__(self, F=MField): 


        # initialize values
        self.F   = F
        self.b   = np.zeros([3, 1])
        self.A_1 = np.eye(3)
        
    def run(self):
        data = np.loadtxt(f"{os.path.dirname(__file__)}/mag_out.txt",delimiter=',')
        print("shape of data:",data.shape)
        print("First 5 rows raw:\n", data[:5])
        
        # ellipsoid fit
        s = np.array(data).T
        M, n, d = self.__ellipsoid_fit(s)

        # calibration parameters
        M_1 = linalg.inv(M)
        self.b = -np.dot(M_1, n)
        self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))
        
        print("Soft iron transformation matrix:\n",self.A_1)
        print("Hard iron bias:\n", self.b)

        result = [] 
        for row in data: 
        
            # subtract the hard iron offset
            xm_off = row[0]-self.b[0]
            ym_off  = row[1]-self.b[1]
            zm_off  = row[2]-self.b[2]
            
            #multiply by the inverse soft iron offset
            xm_cal = xm_off *  self.A_1[0,0] + ym_off *  self.A_1[0,1]  + zm_off *  self.A_1[0,2] 
            ym_cal = xm_off *  self.A_1[1,0] + ym_off *  self.A_1[1,1]  + zm_off *  self.A_1[1,2] 
            zm_cal = xm_off *  self.A_1[2,0] + ym_off *  self.A_1[2,1]  + zm_off *  self.A_1[2,2] 

            result = np.append(result, np.array([xm_cal, ym_cal, zm_cal]) )#, axis=0 )
            #result_hard_iron_bias = np.append(result, np.array([xm_off, ym_off, zm_off]) )

        result = result.reshape(-1, 3)
                
        print("First 5 rows calibrated:\n", result[:5])
        #np.savetxt('out.txt', result, fmt='%f', delimiter=' ,')

        offsets = [float(self.b[0]), float(self.b[1]), float(self.b[2]), float(self.A_1[0,0]), float(self.A_1[1,0]), float(self.A_1[2,0]), float(self.A_1[0,1]), float(self.A_1[1,1]), float(self.A_1[2,1]), float(self.A_1[0,2]), float(self.A_1[1,2]), float(self.A_1[2,2])]
        print("*************************" )        
        print("code to paste : " )
        print("*************************" )  
        print("hard_iron_bias_x = ", float(self.b[0]))
        print("hard_iron_bias_y = " , float(self.b[1]))
        print("hard_iron_bias_z = " , float(self.b[2]))
        print("\n")
        print("soft_iron_bias_xx = " , float(self.A_1[0,0]))
        print("soft_iron_bias_xy = " , float(self.A_1[1,0]))
        print("soft_iron_bias_xz = " , float(self.A_1[2,0]))
        print("\n")
        print("soft_iron_bias_yx = " , float(self.A_1[0,1]))
        print("soft_iron_bias_yy = " , float(self.A_1[1,1]))
        print("soft_iron_bias_yz = " , float(self.A_1[2,1]))
        print("\n")
        print("soft_iron_bias_zx = " , float(self.A_1[0,2]))
        print("soft_iron_bias_zy = " , float(self.A_1[1,2]))
        print("soft_iron_bias_zz = " , float(self.A_1[2,2]))
        print("\n")

        f = open(f"{os.path.dirname(__file__)}/calibOffsets.txt", "w")
        for i in offsets:
            f.write(str(i)+'\n')
        f.close()
        os.remove(f"{os.path.dirname(__file__)}/mag_out.txt")
        
    def __ellipsoid_fit(self, s):
        ''' Estimate ellipsoid parameters from a set of points.

            Parameters
            ----------
            s : array_like
              The samples (M,N) where M=3 (x,y,z) and N=number of samples.

            Returns
            -------
            M, n, d : array_like, array_like, float
              The ellipsoid parameters M, n, d.

            References
            ----------
            .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
               fitting," in Geometric Modeling and Processing, 2004.
               Proceedings, vol., no., pp.335-340, 2004
        '''

        # D (samples)
        D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                      2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                      2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]

        # C (Eq. 8, k=4)
        C = np.array([[-1,  1,  1,  0,  0,  0],
                      [ 1, -1,  1,  0,  0,  0],
                      [ 1,  1, -1,  0,  0,  0],
                      [ 0,  0,  0, -4,  0,  0],
                      [ 0,  0,  0,  0, -4,  0],
                      [ 0,  0,  0,  0,  0, -4]])

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)

        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0: v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

        # quadric-form parameters
        M = np.array([[v_1[0], v_1[3], v_1[4]],
                      [v_1[3], v_1[1], v_1[5]],
                      [v_1[4], v_1[5], v_1[2]]])
        n = np.array([[v_2[0]],
                      [v_2[1]],
                      [v_2[2]]])
        d = v_2[3]

        return M, n, d

class Calibrate:

    def __init__(self, sensor, sampleSize=3000, animate=False):
        self.lis3mdl = sensor
        self.mag_x = deque(maxlen=sampleSize)
        self.mag_y = deque(maxlen=sampleSize)
        self.mag_z = deque(maxlen=sampleSize)
        self.sampleSize = sampleSize
        self.animate = animate
        self.cnt=0
        if self.animate:
            self.fig, self.ax = plt.subplots(1, 1)
            self.ax.set_aspect(1)
    
    def startCalib(self):
        if self.animate:
            self.anim = animation.FuncAnimation(self.fig, self.animateData,interval=200)
            plt.show()
        else:
            while self.cnt<=self.sampleSize:
                self.saveData()
        self.finishCalib()

    def animateData(self, i):
        self.saveData()
        # Clear all axis
        self.ax.cla()
        # Display the sub-plots
        self.ax.scatter(self.mag_x, self.mag_y, color='r')
        self.ax.scatter(self.mag_y, self.mag_z, color='g')
        self.ax.scatter(self.mag_z, self.mag_x, color='b')
        plt.pause(0.01)
        if self.cnt >= self.sampleSize:
            self.anim.event_source.stop()
            plt.close()

    def saveData(self):
        for _ in range(30):
            ret = self.lis3mdl.get_magnetometer_raw()
            if not ret:
                continue
            x = round((ret[0]/6842)*100,3) #in microTesla
            y = round((ret[1]/6842)*100,3)
            z = round((ret[2]/6842)*100,3)
            
            self.mag_x.append(x)
            self.mag_y.append(y)
            self.mag_z.append(z)
            time.sleep(1/70)
            self.cnt+=1
        print(self.cnt)

    def finishCalib(self):
        print("Finished collecting data, now writing to file\n")
        f = open(f"{os.path.dirname(__file__)}/mag_out.txt", "w")
        for i in range(self.sampleSize):
            toWrite = f"{self.mag_x[i]},{self.mag_y[i]},{self.mag_z[i]}"
            f.write(toWrite+'\n')
            pass
        f.close()
        time.sleep(1)
        print("Finished saving data, starting calibration math\n")
        time.sleep(3)
        Magnetometer().run()
        print("Saved data to calibOffsets.txt")

# if __name__ == "__main__":
#     lis3mdl = LIS3MDL()
#     lis3mdl.enable()
#     tempObject = Calibrate(lis3mdl, 200,False)
#     tempObject.startCalib()

