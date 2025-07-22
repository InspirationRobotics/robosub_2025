import time
import threading
import rospy
from std_msgs.msg import Float64

# use module pyserial (not serial)

import serial
from auv.utils import deviceHelper


# Requires FOG USB to be connected to 
# lower-right USB port on Jetson to work

fog_port = deviceHelper.dataFromConfig("fog")

class FOG:

    """Allows for tracking of angular movement"""

    def __init__(self, port='/dev/ttyUSB0'):
        """Initialize the serial connection"""
        rospy.init_node("FOG", anonymous=True)
        # Make serial stuff???
        self.ser = self._setupSerial(fog_port)
        self.readData = False

        self.xDataNames = ["temp", "supply_voltage", "sld_curr", "diag_sig", "angle_deg"]
        self.data = {}
        self.parsed_data = {}

        self.samples = 200
        self.count = 0
        self.angle_sum = 0

        self.cal_time = 10 # seconds
        self.cal_sum = 0
        self.cal_count = 0

        # Previous was 0.0727
        self.integration_factor = 0.0767 # Determined empirically to be 0.0767 deg/sec/mV (should be close to 1/SF)
        self.integrated_sum = 0
        self.bias = 0
        self.pub_fog = rospy.Publisher("auv/devices/fog", Float64, queue_size=10)
        self.pub_ang = rospy.Publisher("auv/devices/fogang", Float64, queue_size=10)

    def _setupSerial(self, p : str) -> serial.Serial:
        """Make a serial connection to store in self.ser,
        look more stuff up"""
        # TODO: Look up serial module, tutorials, etc.
        # Example of Serial use: https://www.youtube.com/watch?v=WV4U51TlRaQ&list=PLb1SYTph-GZJb1CFM7ioVY9XJYlPVUBQy&index=2&pp=iAQB

        return serial.Serial(
            port=p, 
            baudrate=921600, 
            parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE, 
            bytesize=serial.EIGHTBITS)
    
    def start(self, func = None):
        """Wipes parameters, sets boolean self.readData to True,
        initializes a thread of self._read_fog and starts running it"""
        # Threads exist to allow multiple processes to run simultaneously,
        # using downtime in one process to run the others. More detailed
        # info available at https://www.youtube.com/watch?v=A_Z1lgZLSNc

        self.reset_params()
        self.readData = True
        self.read_thread =  threading.Thread(target=self._read_fog, args=(func,))
        self.read_thread.start()

    def reset_params(self):
        """Change counter variables back to zero, wipe
        data, and record the current time"""
        self.data = {}
        self.parsed_data = {}

        self.count = 0
        self.angle_sum = 0

        self.integrated_sum = 0
        self.prev_time = time.time()

        self.cal_sum = 0
        self.cal_count = 0
    
    def reset_bias(self):
        """Change self.bias to zero"""
        self.bias = 0

    def calibrate(self):
        print("Calibrating FOG... DO NOT TOUCH")
        self.reset_params()
        self.start(self._cal_fog_angle_data)
        time.sleep(self.cal_time)
        self.bias = self.cal_sum/(self.cal_count + 0.00001)
        self.stop_read()
        print(f"Calibration complete. Bias: {self.bias}")

    def _handle_checksum(self, data):
        if len(data) != 8:
            return False
        sum = 0
        for i in range(1, len(data) - 2):
            try:
                sum += int(data[i], 16)
            except:
                return False
        high_cs = int(data[6], 16)  # second to last is high byte
        low_cs = int(data[7], 16)
        combined_value = (high_cs << 8) | low_cs
        return combined_value == sum
    
    def _twos_complement(self, value):
        MODULO = 1 << 24
        #MAX_VALUE = (1 << 23) - 1
        #value = int(hexstr, 16)
        if value & (1 << 23):
            value -= MODULO
        return value

    def _cal_fog_angle_data(self, curr_line, PLACEHOLDER):
        """Counterclockwise angles are negative, clockwise 
        angles are positive. Angles in degrees (not radians)"""
        # Calibrate the angle data
        if not self._handle_checksum(curr_line):
            return
        angle_data = int(curr_line[2], 16) << 16 | int(curr_line[3], 16) << 8 | int(curr_line[1], 16)
            # Converts to 24 bit signed integer and adds to the sum
        self.cal_sum += self._twos_complement(angle_data)
        self.cal_count += 1

    def _parse_fog_data(self, curr_line, prev_line):
        # 0 is the sync bit, 1-3 is the angle data (low high middle), 
        # 4 is the packet_count, 5 is the xData (even count is high, 
        # odd count is low), 6 checksum high, 7 checksum low

        # Checksum verification
        if not self._handle_checksum(curr_line) or not self._handle_checksum(prev_line):
            return
        
        # Packet count
        packet_count = int(curr_line[4], 16)

        # Angle data
        angle_data = int(curr_line[2], 16) << 16 | int(curr_line[3], 16) << 8 | int(curr_line[1], 16)
            # Converts to 24 bit signed integer and adds to the sum
        self.angle_sum += (self._twos_complement(angle_data)-self.bias)
        #print(f"Angle: {self.twos_complement(angle_data)} Raw: {curr_line[1]} {curr_line[2]} {curr_line[3]}")
            # Average the angle data
        self.count += 1
        if self.count >= self.samples:
            angle_mv = (self.angle_sum/self.samples)*(2.5/(2**23)) # Converts to mV
            angle_deg_sec = angle_mv*self.integration_factor
            self.integrated_sum += angle_deg_sec*(time.time() - self.prev_time)
            self.parsed_data["angle_deg"] = self.integrated_sum
            self.pub_fog.publish(self.integrated_sum)
            self.pub_ang.publish(angle_deg_sec)
            print(self.integrated_sum)
            # time.sleep(0.1)
            self.prev_time = time.time()
            self.angle_sum = 0
            self.count = 0

        # XData
        if packet_count % 2 != 0 and packet_count < 8:
            xData = int(prev_line[5], 16) << 8 | int(curr_line[5], 16)
            self.data[self.xDataNames[packet_count//2]] = xData

        # Translate the data to the correct units
        self._translate_data()

    def _translate_data(self):
        if "temp" in self.data.keys():
            self.parsed_data["temp"] = self.data["temp"]*(250/(2**15)) - 50 
        if "supply_voltage" in self.data.keys():
            self.parsed_data["supply_voltage"] = self.data["supply_voltage"]*(10/(2**15))
        if "sld_curr" in self.data.keys():
            self.parsed_data["sld_curr"] = self.data["sld_curr"]*(0.25/(2**15))
        if "diag_sig" in self.data.keys():
            self.parsed_data["diag_sig"] = self.data["diag_sig"]*(2.5/(2**15))

    def _read_fog(self, func = None):
        """"""
        # see sample.txt for example data

        # make lines, which will be lists of 8 string elements. The first
        # element is always b'\xdd', the others are two hex digits
        line = []
        prev_line = []

        # Execute while self.readData and self.ser.in_waiting are True
        while self.readData:
            while self.ser.in_waiting:
                if not self.readData: 
                    break

                # Read one byte from the serial connection
                byte = self.ser.read(1)

                # For a given line, append bytes as string elements to a given line (else)
                # unless b'\xdd' is encountered (proceed to if), whereby if the line
                # list has 8 elements, and either the first line is being recorded
                # or the fifth byte of the current line isn't the same as the previous line,
                # run a function to parse the fog data (or self._parse_fog_data if none are provided).
                # Save the current line to the previous line and start a new line of bytes with
                # the b'\xdd' byte

                if byte == b'\xdd':
                    if len(line) == 8 and (prev_line == [] or prev_line[4] != line[4]):
                        if func is None: 
                            self._parse_fog_data(line, prev_line)
                        else: 
                            func(line, prev_line)
                        prev_line = line
                    line = [byte.hex()]
                else:
                    line.append(byte.hex())
    
    def publish_reading(self, reading):
        fog_reading = Float64()
        fog_reading.data = reading
        self.pub_fog.publish(fog_reading)

    def stop_read(self):
        self.readData = False
        self.read_thread.join()
        print("FOG read thread stopped.")

    def close(self):
        if self.readData: self.stop_read()
        self.ser.close()
        print("FOG serial port closed.")

if __name__ == "__main__":
    fog = FOG(fog_port)
    fog.calibrate()

    # --------- Calibration metrics for 60 seconds ------------
    # fog.start()
    
    # startTime = time.time()
    # startTime2 = time.time()
    # times = []

    # time.sleep(0.1)
    # first = fog.parsed_data["angle_deg"] if "angle_deg" in fog.parsed_data.keys() else "No data"
    # while time.time() - startTime < 60:
    #     # Record every 10 seconds
    #     if time.time() - startTime2 > 10:
    #         times.append(fog.parsed_data["angle_deg"])
    #         startTime2 = time.time()
    #     time.sleep(0.1)
    # last = fog.parsed_data["angle_deg"]

    # for i in range(len(times)):
    #     print(f"Time {(i+1)*10}: {times[i]-first}")

    # print(f"First: {first} Last: {last} Difference: {last-first}")

    # fog.stop_read()
    # ---------------------------------------------------------

    print("Now just running for 30 seconds")

    fog.start()

    startTime = time.time()
    for i in range(30):
        print(i+1)
        time.sleep(1)
    fog.stop_read()
    print("Stopping read thread")
    fog.close()
