"""
altimu10v5: Main module

Copyright 2017, Svetoslav Kuzmanov
Licensed under MIT.
"""
from .lsm6ds33 import LSM6DS33
from .lis3mdl import LIS3MDL
from .lps25h import LPS25H


class IMU(object):
    """Set up and control Pololu's AltIMU-10v5."""

    def __init__(self):
        super(IMU, self).__init__()
        self.lsm6ds33 = LSM6DS33()
        self.gyroAccelEnabled = False
        self.lis3mdl = LIS3MDL()
        self.barometerEnabled = False
        self.lps25h = LPS25H()
        self.magnetometerEnabled = False

    def __del__(self):
        del self.lsm6ds33
        del self.lis3mdl
        del self.lps25h

    def enable(self, gyroAccel=True, barometer=True, magnetometer=True):
        """Enable the given devices."""

        if gyroAccel:
            self.lsm6ds33.enable()
            self.gyroAccelEnabled = True
        if barometer:
            self.lps25h.enable()
            self.barometerEnabled = True
        if magnetometer:
            self.lis3mdl.enable()
            self.magnetometerEnabled = True

    def get_complementary_angles(self, delta_t=0.05):
        """Calculate combined angles of accelerometer and gyroscope
        using a complementary filter.
        """
        if not self.gyroAccelEnabled:
            raise (Exception("Gyroscope and accelerometer are not enabled!"))

        self.complementary_angles = [0, 0]
        complementary_filter_constant = 0.98

        accel_angles = self.lsm6ds33.get_accelerometer_angles()
        gyro_angular_velocity = self.lsm6ds33.get_gyro_angular_velocity()

        self.complementary_angles[0] = (
            complementary_filter_constant * (self.complementary_angles[0] + (gyro_angular_velocity[0] * delta_t))
            + (1 - complementary_filter_constant) * accel_angles[0]
        )
        self.complementary_angles[1] = (
            complementary_filter_constant * (self.complementary_angles[1] + (gyro_angular_velocity[1] * delta_t))
            + (1 - complementary_filter_constant) * accel_angles[1]
        )

        return self.complementary_angles
