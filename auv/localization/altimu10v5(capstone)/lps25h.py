# -*- coding: utf-8 -*-

"""Python module for LPS25H digital barometer.
This module is for the Raspberry Pi computer helps interface the LPS25H
digital barometer.The library makes it easy to read the raw barometer
through I²C interface.

The datasheet for the LPS25H is available at
[https://www.pololu.com/file/download/LPS25H.pdf?file_id=0J761]
"""

from .i2c import I2C
from .constants import *


class LPS25H(I2C):
    """Set up and access LPS25H digital barometer."""

    # Output registers used by the barometer sensor
    barometer_registers = [
        LPS25H_PRESS_OUT_XL,  # lowest byte of pressure value
        LPS25H_PRESS_OUT_L,  # low byte of pressure value
        LPS25H_PRESS_OUT_H,  # high byte of pressure value
    ]

    def __init__(self, bus_id=8):
        """Set up and access LPS25H digital barometer."""

        super(LPS25H, self).__init__(bus_id)
        self.is_barometer_enabled = False

    def __del__(self):
        """Clean up."""
        try:
            # Power down barometer
            self.write_register(LPS25H_ADDR, LPS25H_CTRL_REG1, 0x00)
            super(LPS25H, self).__del__()
        except:
            pass

    def enable(self):
        """Enable and set up the LPS25H barometer."""
        # Power down device first
        self.write_register(LPS25H_ADDR, LPS25H_CTRL_REG1, 0x00)

        # Output data rate 12.5Hz
        # binary value -> 10110000, hex value -> 0xb0
        self.write_register(LPS25H_ADDR, LPS25H_CTRL_REG1, 0xB0)

        self.is_barometer_enabled = True

    def get_barometer_raw(self):
        """Return the raw barometer sensor data."""
        # Check if barometer has been enabled
        if not self.is_barometer_enabled:
            raise (Exception("Barometer is not enabled"))

        return self.read_1d_sensor(LPS25H_ADDR, self.barometer_registers)
    
if __name__ == "__main__":
    # Collect raw barometer data
    barometer = LPS25H()
    barometer.enable()
    try:
        while True:
            # Get barometer data
            barometer_data = barometer.get_barometer_raw()
            print(barometer_data)
            barometer.log_sensor_data("Barometer", barometer_data,"Barometer-LPS25H.csv")
            

    except KeyboardInterrupt:
        print("KeyboardInterrupt, killing program")
        del(barometer)
