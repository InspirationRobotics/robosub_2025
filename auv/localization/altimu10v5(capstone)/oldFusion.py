import math
import statistics
import time
import numpy as np
from auv.device.compass.altimu10v5.lis3mdl import LIS3MDL
from auv.device.compass.altimu10v5.lsm6ds33 import LSM6DS33

lsm6ds33 = LSM6DS33(1)
lis3mdl = LIS3MDL(1)

lsm6ds33.enable()
lis3mdl.enable()
#lis3mdl.calibrate(3000, True)

Gyro_pitch = 0
Gyro_roll = 0
Gyro_yaw = 0
Gyro_pitch_output=Gyro_roll_output=Mag_x_dampened=Mag_y_dampened=1
rate = 50 #hz
degToRad = math.pi / 180
radTodeg = 180 / math.pi

while True:
    time.sleep(1/rate)
    accel = lsm6ds33.get_accelerometer_angles()
    gyro = lsm6ds33.get_gyro_angular_velocity()
    mag = lis3mdl.get_magnetometer_calibrated()
    #heading = (math.atan2(mag[1], mag[0]) * 180) / math.pi
    #print(heading)
    
    '''
    ---------------------------
    Adjust Gyro_xyz signs for:
    ---------------------------
    Pitch (Nose - up) = +ve reading
    Roll (Right - wing down) = +ve reading
    Yaw (Clock - wise rotation)  = +ve reading
    '''
    # integrate gyro readings to get angle
    Gyro_pitch += -gyro[0] / rate # Integrate the raw Gyro_x readings
    Gyro_roll += -gyro[1]  / rate  # Integrate the raw Gyro_y readings
    Gyro_yaw -= gyro[2] /rate  # Integrate the raw Gyro_z readings
    
    #print(Gyro_pitch, Gyro_yaw, Gyro_roll)

    # Compensate pitch and roll for gryo yaw
    #Gyro_pitch -= Gyro_roll * math.sin(gyro[2] * degToRad)
    #Gyro_roll += Gyro_pitch * math.sin(gyro[2] * degToRad)

    #print(Gyro_pitch, Gyro_roll, Gyro_yaw)
    
    # // ----- Zero any residual accelerometer readings
    # Place the accelerometer on a level surface
    # Adjust the following two values until the pitch and roll readings are zero
    Accel_roll = accel[0] 
    Accel_pitch = accel[1]
    #Accel_pitch -= -0.2 #Accelerometer calibration value for pitch
    #Accel_roll -= 1.1 #Accelerometer calibration value for roll
    
    # // ----- Sync gyro to accelerometer
    Gyro_pitch = Gyro_pitch * 0.9996 + Accel_pitch * 0.0004;  #Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    Gyro_roll = Gyro_roll * 0.9996 + Accel_roll * 0.0004;   #Correct the drift of the gyro roll angle with the accelerometer roll angle
    
    # // ----- Dampen the pitch and roll angles
    Gyro_pitch_output = Gyro_pitch_output * 0.9 + Gyro_pitch * 0.1;   # Take 90% of the output pitch value and add 10% of the raw pitch value
    Gyro_roll_output = Gyro_roll_output * 0.9 + Gyro_roll * 0.1;      # Take 90% of the output roll value and add 10% of the raw roll value

    # // ----- Calculate the new tilt compensated values
    Mag_pitch = Gyro_roll_output * degToRad
    Mag_roll = Gyro_pitch_output * degToRad

    # // ----- Apply the standard tilt formulas
    Mag_x, Mag_y, Mag_z = mag
    #Mag_x_hor = Mag_x * math.cos(Mag_pitch) + Mag_y * math.sin(Mag_roll) * math.sin(Mag_pitch) - Mag_z * math.cos(Mag_roll) * math.sin(Mag_pitch)
    #Mag_y_hor = Mag_y * math.cos(Mag_roll) + Mag_z * math.sin(Mag_roll)

    magXcomp = Mag_x*math.cos(Mag_pitch)+Mag_z*math.sin(Mag_pitch)
    magYcomp = Mag_x*math.sin(Mag_roll)*math.sin(Mag_pitch)+Mag_y*math.cos(Mag_roll)-Mag_z*math.sin(Mag_roll)*math.cos(Mag_pitch)
    heading = 180*math.atan2(magYcomp,magXcomp)/math.pi
    print(heading)

    # // ----- Dampen any data fluctuations
    #Mag_x_dampened = Mag_x_dampened * 0.9 + Mag_x_hor * 0.1
    #Mag_y_dampened = Mag_y_dampened * 0.9 + Mag_y_hor * 0.1

    # // ----- Calculate the heading
    #heading = math.atan2(Mag_y_hor, Mag_x_hor) * radTodeg
    #print(heading)

    # -----------------------------------------------------------------------------------------------
    # E = np.cross(mag, accel)
    # E = E / np.linalg.norm(E)
    # N = np.cross(accel, E)
    # N = N / np.linalg.norm(N)
    # heading = math.atan2(np.dot(E, [1, 0, 0]), np.dot(N, [1, 0, 0]) * 180 / math.pi)
    # print(heading)
