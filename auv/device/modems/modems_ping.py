"""
Pings the other sub through the serial connection and retrieves the status of the other sub.
"""

from . import modems_node
from ...utils.deviceHelper import variables

import time

modem = modems_node.Modem(auto_start=False) # Start a modem 
config = variables
# If the sub to ping is Graey, address will be 111, if the sub to ping is Onyx, address will be 222
addr = 111
if (variables.get("sub", "graey") == "onyx"):
    addr = 222

count = 0
maxPings = 7 # Maximum number of pings before timeout
data = None


while (count < maxPings and data == None):
    temp = modem.ping_status(111) # Ping Graey 
    if "#R" in temp: 
        data = temp
    modem.led.on_send_msg()
    time.sleep(0.5)
    count += 1

print("Ping Status: ", data)
#modem.stop()