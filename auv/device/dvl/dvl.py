# Documentation for Teledyne available at https://drive.google.com/file/d/1xniDtjYIJhaFOhWaSj4tj6RxBdN5inx2/view
# See page 93 for specs

import rospy
import math
import signal
import threading
import time
import json

import numpy as np

import csv
from datetime import datetime

import serial
from geometry_msgs.msg import Vector3Stamped, PointStamped

from auv.device.dvl import dvl_tcp_parser
from auv.utils import deviceHelper


class DVL:
    """DVL class to enable position estimation"""
    
    def __init__(self, autostart=True, compass=False, test=False):
        rospy.init_node("dvl", anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz
        
        self.vel_pub = rospy.Publisher('/auv/devices/dvl/velocity', Vector3Stamped, queue_size=10)


        # TODO: Review if we need a position publisher

        self.pos_pub = rospy.Publisher('/auv/devices/dvl/position', PointStamped, queue_size=10)
        
        self.test = test
        if not self.test:
            self.dvlPort = deviceHelper.dataFromConfig("dvl")
            print(self.dvlPort)
            self.sub = deviceHelper.variables.get("sub")
            print(f"[DEBUG] Sub is {self.sub}")
            if self.sub == "onyx":
                self.ser = serial.Serial(
                    port=self.dvlPort,
                    baudrate=115200,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                )

                self.dvl_rot = math.radians(45)
                self.ser.isOpen()
                self.ser.reset_input_buffer()
                self.ser.send_break()
                time.sleep(1)
                startPing = "CS"
                self.ser.write(startPing.encode())
                time.sleep(2)
                self.read = self.read_onyx

            elif self.sub == "graey":
                # autostart = False
                self.read = self.read_graey
                self.dvl_rot = math.radians(0)
            else:
                raise ValueError(f"Invalid sub {self.sub}")

        self.enable_compass = compass

        self.__running = False
        self.__thread_vel = None
        self.prev_time = None
        self.current_time = None
        
        # sensor error
        self.compass_error = math.radians(1.0)  # rad/s
        
        self.error = [0, 0, 0]  # accumulated error

        # NORTH = 0, EAST = pi/2, SOUTH = pi, WEST = 3pi/2
        self.compass_rad = None  # rad

        self.vel_rot = [0, 0, 0]  # rotated velocity vector

        # position in meters
        # X = lateral
        # Y = forward
        # Z = {"Onyx": "distance from bottom",
        # "Graey": "Distance from surface"}
        self.position = [0, 0, 0]  


        self.is_valid = False
        self.data_available = False

        # stores position and error history for context manager
        self.position_memory = []
        self.error_memory = []

        # DVL data packet. Must put this in __init__ to allow for the
        # time to accumulate in read_graey
        self.graey_data = {
            "time": 0,  # seconds
            "vx": 0,  # m/s
            "vy": 0,  # m/s
            "vz": 0,  # m/s
            "error": 0, # m/s - is placeholder for dynamic value
            "valid": False,  # boolean
            }

        if autostart:
            self.start()

    def __parseLine(self, line):
        """Parse line"""
        return line.decode("utf-8").replace(" ", "").replace("\r\n", "").split(",")

    def read_graey(self):
        """Get velocity from graey"""

        # Useful documentation for JSON strings: 
        # https://www.geeksforgeeks.org/convert-json-to-dictionary-in-python/

        # On Graey run python3 -m (filepath) velocity -i 192.168.2.10
        try:
            data_iterator = dvl_tcp_parser.main()
            for line in data_iterator:
                line = json.loads(line)

                # In Graey, x-axis is forward and y-axis is lateral,
                # to be consistent w/ Onyx we will switch them here
                self.graey_data["time"] += float(line["time"]) / 1000
                self.graey_data["vx"] = float(line["vy"])
                self.graey_data["vy"] = float(line["vx"])
                self.graey_data["vz"] = float(line["vz"])
                self.graey_data["error"] = float(line["fom"])
                self.graey_data["valid"] = line["velocity_valid"]
                return self.graey_data
        except:
            print("I threw an exception!")
            data = None
        return data

    def read_onyx(self):
        """Get velocity from onyx"""

        while not self.ser.in_waiting:
            # take a nap :)
            time.sleep(0.01)
            # print("[DEBUG] Serial is not working!!!")

        data = {
            "time": 0,  # seconds
            "vx": 0,  # m/s
            "vy": 0,  # m/s
            "vz": 0,  # m/s
            "error": 0.002, # m/s - see Teledyne Documentation note above
            "valid": False,  # boolean
        }
        SA = self.__parseLine(self.ser.readline())
        if SA[0] != ":SA":
            return None

        TS = self.__parseLine(self.ser.readline())
        WI = self.__parseLine(self.ser.readline()) # unused
        BI = self.__parseLine(self.ser.readline())
        WS = self.__parseLine(self.ser.readline()) # unused
        BS = self.__parseLine(self.ser.readline())
        WE = self.__parseLine(self.ser.readline()) # unused
        BE = self.__parseLine(self.ser.readline())  # unused
        WD = self.__parseLine(self.ser.readline()) # unused
        BD = self.__parseLine(self.ser.readline())


        try:
            # data["Attitude"] = [float(SA[1]), float(SA[2]), float(SA[3])]
            # data["Salinity"] = float(TS[2])
            # data["Temp"] = float(TS[3])
            # data["Transducer_depth"] = float(TS[4])
            # data["Speed_of_sound"] = float(TS[5])
            # data["Result_code"] = TS[6]
            # data["DVL_velocity"] = [int(BI[1]), int(BI[2]), int(BI[3]), int(BI[4])]
            # data["isDVL_velocity_valid"] = BI[5] == "A"
            # data["AUV_velocity"] = [int(BS[1]), int(BS[2]), int(BS[3])]
            # data["isAUV_velocity_valid"] = BS[4] == "A"
            # data["Distance_from_bottom"] = float(BD[4])
            # data["Time_since_valid"] = float(BD[5])

            centi = int(TS[1][12:14]) * 0.01
            seconds = int(TS[1][10:12])
            minutes = int(TS[1][8:10]) * 60
            hours = int(TS[1][6:8]) * 60 * 60
            t = hours + minutes + seconds + centi

            # this is the only data we need
            data["time"] = t
            data["vx"] = int(BS[1]) / 1000
            data["vy"] = int(BS[2]) / 1000
            data["vz"] = int(BS[3]) / 1000
            data["valid"] = BS[4] == "A"
        except:
            print("I failed")
            data = None
        # print("[DEBUG] Data in read_onyx: ", data)
        return data

    def process_packet_compass(self, packet):
        """integrate velocity into position"""

        vel = [packet.get("vx", 0), packet.get("vy", 0), packet.get("vz", 0)]
        self.current_time = packet.get("time", 0)  # seconds
        self.dvl_error = packet.get("error", 0)

        if self.prev_time is None or self.compass_rad is None:
            self.prev_time = self.current_time
            print("[WARN] DVL not ready, waiting for compass or some more sample")
            return False

        dt = self.current_time - self.prev_time
        if dt < 0:
            print("[WARN] DVL time error, skipping")
            return False

        self.is_valid = packet["valid"]
        if not self.is_valid:
            # print("[WARN] DVL velocity not valid, skipping")
            return False

        self.prev_time = self.current_time

        # rotate velocity vector using compass heading
        # plus 45 degrees
        # X = lateral, Y = forward, Z = vertical
        self.vel_rot = [
            vel[0] * math.cos(self.compass_rad + self.dvl_rot) + vel[1] * math.sin(self.compass_rad + self.dvl_rot),
            vel[1] * math.cos(self.compass_rad + self.dvl_rot) - vel[0] * math.sin(self.compass_rad + self.dvl_rot),
            vel[2],
        ]

        # integrate velocity to position with respect to time
        self.position = [
            self.position[0] + self.vel_rot[0] * dt,
            self.position[1] + self.vel_rot[1] * dt,
            self.position[2] + self.vel_rot[2] * dt,
        ]

        vel_rot_error = [
            (vel[0] + self.dvl_error) * math.cos(self.compass_rad + self.dvl_rot + self.compass_error)
            + (vel[1] + self.dvl_error) * math.sin(self.compass_rad + self.dvl_rot + self.compass_error),
            (vel[1] + self.dvl_error) * math.cos(self.compass_rad + self.dvl_rot + self.compass_error)
            - (vel[0] + self.dvl_error) * math.sin(self.compass_rad + self.dvl_rot + self.compass_error),
            vel[2] + self.dvl_error,  # we actually have a sensor for depth, so useless
        ]

        # calculate accumulated error
        self.error = [
            self.error[0] + abs(self.vel_rot[0] - vel_rot_error[0]) * dt,
            self.error[1] + abs(self.vel_rot[1] - vel_rot_error[1]) * dt,
            self.error[2] + abs(self.vel_rot[2] - vel_rot_error[2]) * dt,
        ]

        return True

    def process_packet(self, packet):
        """Integrate velocity into position without compass"""

        vel = [packet.get("vx", 0), packet.get("vy", 0), packet.get("vz", 0)]
        self.current_time = packet.get("time", 0)  # seconds
        self.dvl_error = packet.get("error", 0)

        if self.prev_time is None:
            self.prev_time = self.current_time
            print("[WARN] DVL not ready, waiting for some more sample")
            return False

        dt = self.current_time - self.prev_time
        if dt < 0:
            print("[WARN] DVL time error, skipping")
            return False

        self.is_valid = packet["valid"]
        if not self.is_valid:
            # print("[WARN] DVL velocity not valid, skipping")
            return False
        
        # print("[DEBUG] Running the process packet method")
        # print(f"[DEBUG]: Current time is {self.current_time}")
        # print(f"[DEBUG]: Previous time is {self.prev_time}")

        self.prev_time = self.current_time

        # Rotate velocity vector by DVL rotation of 45 degrees

        self.vel_rot = [
            vel[0] * math.cos(self.dvl_rot) + vel[1] * math.sin(self.dvl_rot),
            vel[1] * math.cos(self.dvl_rot) - vel[0] * math.sin(self.dvl_rot),
            vel[2],
        ]

        # integrate velocity to position with respect to time
        self.position = [
            self.position[0] + self.vel_rot[0] * dt,
            self.position[1] + self.vel_rot[1] * dt,
            self.position[2] + self.vel_rot[2] * dt,
        ]

        vel_error = [
            vel[0] + self.dvl_error,
            vel[1] + self.dvl_error,
            vel[2] + self.dvl_error,
        ]

        # calculate accumulated error
        self.error = [
            self.error[0] + abs(vel[0] - vel_error[0]) * dt,
            self.error[1] + abs(vel[1] - vel_error[1]) * dt,
            self.error[2] + abs(vel[2] - vel_error[2]) * dt,
        ]

        return True

    def reset_position(self):
        """Reset position to 0"""
        self.position = [0, 0, 0]
        self.error = [0, 0, 0]

    def update(self): 
        """Update DVL data (runs in a thread)"""
        print("[DEBUG] DVL update() loop started")

        while self.__running and not rospy.is_shutdown():
            vel_packet = self.read()

            if vel_packet is None:
                continue

            if self.enable_compass:
                ret = self.process_packet_compass(vel_packet)
            else:
                ret = self.process_packet(vel_packet)

            self.data_available = ret
            self.rate.sleep()

    def publish_dvl(self, frame_id: str):
        """Publish DVL data to rostopics
        Args:
            - frame_id (str): Frame ID containing name of AUV"""
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            vel_msg = Vector3Stamped()
            vel_msg.header.stamp = now
            vel_msg.header.frame_id = frame_id

            vel_msg.vector.x = self.vel_rot[0]
            vel_msg.vector.y = self.vel_rot[1]
            vel_msg.vector.z = self.vel_rot[2]
            self.vel_pub.publish(vel_msg)

            pos_msg = PointStamped()
            pos_msg.header.stamp = now
            pos_msg.header.frame_id = frame_id
            pos_msg.point.x = self.position[0]
            pos_msg.point.y = self.position[1]
            pos_msg.point.z = self.position[2]
            self.pos_pub.publish(pos_msg)
            self.rate.sleep()
    
    def start(self):
        # ensure not running
        print("[DEBUG] Started successfully")
        if self.__running:
            print("[WARN] DVL already running")
            return

        self.__running = True
        self.__thread_vel = threading.Thread(target=self.update, daemon=True)
        self.__thread_vel.start()

        # Add publisher thread depending on sub
        if self.sub == "graey":
            self.__thread_pub = threading.Thread(target=self.publish_dvl, args=("graey_dvl"), daemon=True)
        elif self.sub == "onyx":
            self.__thread_pub = threading.Thread(target=self.publish_dvl, args=("onyx_dvl"), daemon=True)

        else:
            raise ValueError(f"[ERROR] Unknown sub: {self.sub}")

        self.__thread_pub.start()


    def stop(self):
        self.__running = False
        self.__thread_vel.join()

    def __enter__(self):
        """Context manager for DVL"""
        if not self.__running and not self.test:
            self.start()

        # begin a context, store current position
        self.position_memory.append(self.position)
        self.error_memory.append(self.error)

        # reset position
        self.reset_position()

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Exit context manager"""
        prev_pos = self.position_memory.pop()
        prev_error = self.error_memory.pop()

        # restore previous position and error
        self.position = [
            self.position[0] + prev_pos[0],
            self.position[1] + prev_pos[1],
            self.position[2] + prev_pos[2],
        ]

        self.error = [
            self.error[0] + prev_error[0],
            self.error[1] + prev_error[1],
            self.error[2] + prev_error[2],
        ]

def csvLog(dvl, filename="dvl_log.csv"):
        """
        Logs DVL position and velocity data to a CSV file.
        """
        with open(filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "vx (m/s)", "vy (m/s)", "vz (m/s)", "X (m)", "Y (m)", "Z (m)", "Valid"])

            try:
                while True:
                    time.sleep(0.1)  # sampling delay
                    vel_packet = dvl.read()
                    if vel_packet is None or not vel_packet["valid"]:
                        continue

                    if dvl.enable_compass:
                        dvl.process_packet_compass(vel_packet)
                    else:
                        dvl.process_packet(vel_packet)

                    row = [
                        dvl.current_time,
                        vel_packet["vx"],
                        vel_packet["vy"],
                        vel_packet["vz"],
                        dvl.position[0],
                        dvl.position[1],
                        dvl.position[2],
                        vel_packet["valid"],
                    ]
                    writer.writerow(row)
                    print(f"[LOGGING] {row}")

            except KeyboardInterrupt:
                print("\n[INFO] Logging interrupted. Saving CSV file...")
                print(f"[INFO] Data saved to {filename}")
if __name__ == '__main__':
    # Make a new dvl instance
    try:
        dvl1 = DVL()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"dvl_log_{timestamp}.csv"
        csvLog(dvl1, filename)
        rospy.spin()
    
    except KeyboardInterrupt:
        print("\n[INFO] DVL stopped by user.")
        dvl1.stop()
        rospy.shutdown()
        
    finally:
        print("Node exited.")