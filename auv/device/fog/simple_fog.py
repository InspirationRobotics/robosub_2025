#!/usr/bin/env python
import time
import threading
import rospy
from std_msgs.msg import Float64
import serial
from auv.utils import deviceHelper

class FOG:
    def __init__(self, port='/dev/ttyUSB0'):
        rospy.init_node("fog_node", anonymous=True)
        self.ser = self._setupSerial(port)
        self.readData = False
        self.xDataNames = ["temp", "supply_voltage", "sld_curr", "diag_sig", "angle_deg"]
        self.data = {}
        self.parsed_data = {}
        self.samples = rospy.get_param('~samples', 200)
        self.count = 0
        self.angle_sum = 0
        self.cal_time = rospy.get_param('~cal_time', 10)
        self.cal_sum = 0
        self.cal_count = 0
        self.integration_factor = deviceHelper.variables.get("FOG_SCALE_FACTOR", 120)
        self.integrated_sum = 0
        self.bias = 0
        self.pub_fog = rospy.Publisher("auv/devices/fog", Float64, queue_size=10)
        self.calibrate_at_start = True

        if self.calibrate_at_start:
            self.calibrate()

    def _setupSerial(self, p : str) -> serial.Serial:
        return serial.Serial(
            port=p, 
            baudrate=921600, 
            parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE, 
            bytesize=serial.EIGHTBITS,
            timeout=0)  # Non-blocking mode

    def start(self, func=None):
        self.reset_params()
        self.readData = True
        self.read_thread = threading.Thread(target=self._read_fog, args=(func,))
        self.read_thread.daemon = True
        self.read_thread.start()

    def reset_params(self):
        self.data = {}
        self.parsed_data = {}
        self.count = 0
        self.angle_sum = 0
        self.integrated_sum = 0
        self.prev_time = time.time()
        self.cal_sum = 0
        self.cal_count = 0
    
    def reset_bias(self):
        self.bias = 0

    def calibrate(self):
        rospy.loginfo("Calibrating FOG... DO NOT TOUCH")
        self.reset_params()
        self.start(self._cal_fog_angle_data)
        time.sleep(self.cal_time)
        self.bias = self.cal_sum / (self.cal_count + 1e-9)
        self.stop_read()
        rospy.loginfo(f"Calibration complete. Bias: {self.bias}")

    def _handle_checksum(self, data):
        if len(data) != 8:
            return False
        total = 0
        for i in range(1, len(data) - 2):
            try:
                total += int(data[i], 16)
            except:
                return False
        high_cs = int(data[6], 16)
        low_cs = int(data[7], 16)
        combined_value = (high_cs << 8) | low_cs
        return combined_value == total
    
    def _twos_complement(self, value):
        MODULO = 1 << 24
        if value & (1 << 23):
            value -= MODULO
        return value

    def _cal_fog_angle_data(self, curr_line, PLACEHOLDER):
        if not self._handle_checksum(curr_line):
            return
        angle_data = int(curr_line[2], 16) << 16 | int(curr_line[3], 16) << 8 | int(curr_line[1], 16)
        self.cal_sum += self._twos_complement(angle_data)
        self.cal_count += 1

    def _parse_fog_data(self, curr_line, prev_line):
        if not (self._handle_checksum(curr_line) and self._handle_checksum(prev_line)):
            return
        
        packet_count = int(curr_line[4], 16)
        angle_data = int(curr_line[2], 16) << 16 | int(curr_line[3], 16) << 8 | int(curr_line[1], 16)
        self.angle_sum += (self._twos_complement(angle_data) - self.bias)
        self.count += 1
        
        if self.count >= self.samples:
            angle_mv = (self.angle_sum / self.samples) * (2.5 / (2**23))
            angle_deg_sec = angle_mv * self.integration_factor
            current_time = time.time()
            self.integrated_sum += angle_deg_sec * (current_time - self.prev_time)
            self.parsed_data["angle_deg"] = self.integrated_sum
            
            # publishing data
            fog_msg = Float64()
            fog_msg.data = (self.integrated_sum+360)%360
            self.pub_fog.publish(fog_msg)
            
            self.prev_time = current_time
            self.angle_sum = 0
            self.count = 0

        if packet_count % 2 != 0 and packet_count < 8:
            xData = int(prev_line[5], 16) << 8 | int(curr_line[5], 16)
            self.data[self.xDataNames[packet_count//2]] = xData

        self._translate_data()

    def _translate_data(self):
        if "temp" in self.data:
            self.parsed_data["temp"] = self.data["temp"] * (250/(2**15)) - 50 
        if "supply_voltage" in self.data:
            self.parsed_data["supply_voltage"] = self.data["supply_voltage"] * (10/(2**15))
        if "sld_curr" in self.data:
            self.parsed_data["sld_curr"] = self.data["sld_curr"] * (0.25/(2**15))
        if "diag_sig" in self.data:
            self.parsed_data["diag_sig"] = self.data["diag_sig"] * (2.5/(2**15))

    def _read_fog(self, func=None):
        line = []
        prev_line = []
        
        try:
            while self.readData and not rospy.is_shutdown():
                n = self.ser.in_waiting
                if n > 0:
                    data = self.ser.read(n)
                    for b in data:
                        byte = bytes([b])
                        if byte == b'\xdd':
                            if len(line) == 8 and (not prev_line or prev_line[4] != line[4]):
                                if func is None:
                                    self._parse_fog_data(line, prev_line)
                                else:
                                    func(line, prev_line)
                                prev_line = line
                            line = [byte.hex()]
                        else:
                            line.append(byte.hex())
                else:
                    time.sleep(0.001)  # Reduce CPU usage
        except serial.SerialException as e:
            rospy.logerr(f"Serial error: {str(e)}")
        rospy.loginfo("FOG read thread stopped")

    def stop_read(self):
        if self.readData:
            self.readData = False
            if hasattr(self, 'read_thread') and self.read_thread.is_alive():
                self.read_thread.join()

    def close(self):
        self.stop_read()
        if self.ser.is_open:
            self.ser.close()
        rospy.loginfo("FOG serial port closed")

if __name__ == "__main__":
    fog_port = deviceHelper.dataFromConfig("fog")
    fog = FOG(fog_port)
    
    rospy.on_shutdown(fog.close)
    fog.start()
    rospy.spin()