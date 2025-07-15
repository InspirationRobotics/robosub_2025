# -*- coding: utf-8 -*-

"""Python library module for LIS3MDL magnetometer.
This module for the Raspberry Pi computer helps interface the LIS3MDL
magnetometer.The library makes it easy to read the raw magnetometer
through I²C interface.

The datasheet for the LSM6DS33 is available at
[https://www.pololu.com/file/download/LIS3MDL.pdf?file_id=0J1089]
"""

from .i2c import I2C
from .constants import *
from .lis3mdlCalib import Calibrate
import time
import os


class LIS3MDL(I2C):
    """Set up and access LIS3MDL magnetometer."""

    # Output registers used by the magnetometer
    magnetometer_registers = [
        LIS3MDL_OUT_X_L,  # low byte of X value
        LIS3MDL_OUT_X_H,  # high byte of X value
        LIS3MDL_OUT_Y_L,  # low byte of Y value
        LIS3MDL_OUT_Y_H,  # high byte of Y value
        LIS3MDL_OUT_Z_L,  # low byte of Z value
        LIS3MDL_OUT_Z_H,  # high byte of Z value
    ]

    def __init__(self, bus_id=8):
        """Set up I2C connection and initialize some flags and values."""

        super(LIS3MDL, self).__init__(bus_id)
        self.is_magnetometer_enabled = False
        self.basePath = f"{os.path.dirname(__file__)}/calibOffsets.txt"
        print(self.basePath)
        self.is_mag_calibrated = os.path.exists(self.basePath)
        self.offsets = []
        if self.is_mag_calibrated:
            with open(self.basePath) as file:
                for line in file:
                    self.offsets.append(float(line))

    def __del__(self):
        """Clean up."""
        try:
            # Power down magnetometer
            self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x03)
            super(LIS3MDL, self).__del__()
        except:
            pass

    def enable(self):
        """Enable and set up the the magnetometer and determine
        whether to auto increment registers during I2C read operations.
        """

        # Disable magnetometer and temperature sensor first
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, 0x00)
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x03)

        # Enable device in continuous conversion mode
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x00)

        # Initial value for CTRL_REG1
        ctrl_reg1 = 0x00

        # Ultra-high-performance mode for X and Y
        # Output data rate 80Hz
        # Temp Sens enable 0 (1 for enable)
        # ultra high perfomance mode 11
        # data rate 80hz 111
        # fast odr disabled 0
        # self test diabled 0
        # binary value -> 01110000b, hex value -> 0x70 (10 hz)
        # binary value -> 01111100b, hex value -> 0x7C (80hz)
        ctrl_reg1 += 0x7C

        # +/- 4 gauss full scale
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG2, 0x00)

        # Ultra-high-performance mode for Z
        # binary value -> 00001100b, hex value -> 0x0c
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG4, 0x0C)

        self.is_magnetometer_enabled = True

        # Write calculated value to the CTRL_REG1 register
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, ctrl_reg1)
        time.sleep(0.1)

    def calibrate(self, iterations=3000, animation=False):
        """Calibrate the mags raw values."""
        if not self.is_magnetometer_enabled:
            raise (Exception("Magnetometer is not enabled"))
        print("Calibrating Magnetometer...")
        tempObject = Calibrate(self, iterations, animation)
        tempObject.startCalib()
        self.is_mag_calibrated = os.path.exists(self.basePath)
        if self.is_mag_calibrated:
            with open(self.basePath) as file:
                for line in file:
                    self.offsets.append(float(line))
        print("Done Calibrating...")


    def get_magnetometer_raw(self):
        """Return 3D vector of raw magnetometer data."""
        # Check if magnetometer has been enabled
        if not self.is_magnetometer_enabled:
            raise (Exception("Magnetometer is not enabled"))

        return self.read_3d_sensor(LIS3MDL_ADDR, self.magnetometer_registers)

    def get_magnetometer_calibrated(self):
        if not self.is_mag_calibrated:
            raise Exception("Magnetometer not calibrated")
        magCalib = self.get_magnetometer_raw()

        hard_iron_bias_x =  self.offsets[0]
        hard_iron_bias_y =  self.offsets[1]
        hard_iron_bias_z =  self.offsets[2]
        soft_iron_bias_xx =  self.offsets[3]
        soft_iron_bias_xy =  self.offsets[4]
        soft_iron_bias_xz =  self.offsets[5]
        soft_iron_bias_yx =  self.offsets[6]
        soft_iron_bias_yy =  self.offsets[7]
        soft_iron_bias_yz =  self.offsets[8]
        soft_iron_bias_zx =  self.offsets[9]
        soft_iron_bias_zy =  self.offsets[10]
        soft_iron_bias_zz =  self.offsets[11]

        # get values x,y,z and subtract the hard iron offset
        xm_off = magCalib[0] - hard_iron_bias_x
        ym_off = magCalib[1] - hard_iron_bias_y
        zm_off = magCalib[2] - hard_iron_bias_z
        
        # multiply by the inverse soft iron offset 
        xm_cal = xm_off *  soft_iron_bias_xx + ym_off *  soft_iron_bias_yx  + zm_off *  soft_iron_bias_zx
        ym_cal = xm_off *  soft_iron_bias_xy + ym_off *  soft_iron_bias_yy + zm_off *  soft_iron_bias_zy
        zm_cal = xm_off *  soft_iron_bias_xz + ym_off *  soft_iron_bias_yz  + zm_off *  soft_iron_bias_zz
 
        magCalib = [xm_cal, ym_cal, zm_cal]

        return magCalib
