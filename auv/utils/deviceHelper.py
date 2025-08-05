"""
To get the configuration of the devices connected to the sub(thrusters, cameras, etc.)
"""

import os # For interacting with the operating system
import sys # For system-specific variables and functionalities
import platform # For platform-specific functionalities
# from dotenv import dotenv_values # To load environment variables - this isn't used in the file
import json

def load_json(path):
    """Load JSON data from a file and return it, given its file path"""
    # Open the file, load the file
    with open(path, "r") as f:
        data = json.load(f)
    return data

file_dir = os.path.dirname(os.path.abspath(__file__)) # Obtain the file directory path of the current script

# If "nx" is in the platform node name, Onyx is the sub, else Graey (Onyx runs on an Nvidia NX)
# Load the configuration of Onyx/Graey
hostname = platform.node()
print(f"Hostname is: {hostname}")
if "jetson-desktop" in hostname:
    onyx = True 
    variables = load_json(f"{file_dir}/../../config/onyx.json")
else:
    onyx = False
    variables = load_json(f"{file_dir}/../../config/graey.json")

def findFromId(ids):
    """Finding devices based on their IDs"""
    print(f"Starting findFromId for {ids}")
    bash = os.popen("bash /home/inspiration/robosub_2025/auv/utils/usbLink.sh").read() # Read usbLink.sh
    bashSplit = bash.split("\n") # Split output into lines at "\n"
    result = []

    # Iterating over the IDs along w/ their indices
    for id in enumerate(ids):
        result.append("")
        # Iterating over each line in the splitted output
        for line in bashSplit:
            if "/dev/tty" in line: # If the line is written to /dev/tty (controlling terminal associated with the current process)
                line = line.split(" - ") # Look at usbLink.sh to fully understand (line 24)
                # If the ID path corresponds, append the device name of the device to results
                if id[1] in line[1]: 
                    result[id[0]] = line[0] 

    # Remove any empty spots in results, if no devices avaliable, print the USB list from usbLink.sh
    # If there are results, print the device name
    for i in reversed(result):
        if i == "":
            result.remove(i)
    if len(result) == 0:
        print(bash)
        print("Device not found, above is list of all available devices")
        return None
    
    print(f"Find port {result[0]}")
    return result[0]


def findCam(ids):
    """To find cameras based on their IDs"""
    # Read the list of USB devices, split into lines at "\n"
    # NOTE you may want to change this path next year
    bash = os.popen("bash /home/inspiration/robosub_2025/auv/utils/usbLink.sh").read()
    bash = bash.split("\n")
    result = []
    # For each ID, go over each line, and if the device is written to "dev/video", which means it's a camera,
    # then split the line at the "-".
    for id in enumerate(ids):
        result.append("")
        minVal = 100
        for line in bash:
            if "/dev/video" in line:
                line = line.split(" - ")
                # If the ID paths correspond
                if id[1] == line[1]:
                    # Extract the numerical value from the device path
                    val = int(line[0][10:])
                    # If the value is less than the current minimum value, then set the name of the device to the device name
                    # This is important to make sure we get the camera with the lowest numerical value (default) 
                    if val < minVal:
                        minVal = val
                        result[id[0]] = line[0]

    # Remove any empty strings
    for i in reversed(result):
        if i == "":
            result.remove(i)
    return result


def dataFromConfig(name):
    """Obtain the configurations of a device based on the name of the device"""
    # Look at configs of Graey and Onyx for a full picture
    data = None
    usbID = None
    if name == "forwardOak":
        data = variables.get("forwardOak_camera_mxid")
    elif name == "bottomOak":
        data = variables.get("bottomOak_camera_mxid")
    elif name == "poeOak":
        data = variables.get("poeOak_camera_mxid")
    elif name == "forwardUSB":
        data = variables.get("forward_camera_port")
    elif name == "bottomUSB":
        data = variables.get("bottom_camera_port")
    elif name == "pixhawk":
        usbID = variables.get("pixhawk_port")
    elif name == "modem":
        usbID = variables.get("modem_port")
    elif name == "dvl":
        usbID = variables.get("dvl_port")
    elif name == "polulu":
        usbID = variables.get("polulu_port")
    elif name == "teensy":
        usbID = variables.get("teensy_port")
    elif name == "fog":
        usbID = variables.get("fog_port")
    elif name == "vectornav":
        usbID = variables.get("vectornav_port")
    else:
        data = variables.get(name)
        if data == None:
            raise Exception("Invalid Name")
    if data != None:
        return data
    if usbID == None:
        print("id not found")
        return None  # id is not on sub so leave it
    print(f"USB ID: {usbID}")
    return findFromId([usbID])


if __name__ == "__main__":
    """
    When running the script directly, get the configuration of the device 
    using a command line argument that is the name of the device
    """
    if len(sys.argv) > 1:
       print(dataFromConfig(sys.argv[1]))
    else:
       print(dataFromConfig("pixhawk"))
