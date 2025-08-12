#!/bin/bash

PRODUCT=$(sudo lshw -json | jq '.product') || PRODUCT=$(sudo lshw -json | jq '.[].product')

# cd into absolute path of robosub repo
AUV_PATH=$(dirname $(dirname $(realpath $0)))
cd $AUV_PATH

if [[ $PRODUCT == *"Xavier"* ]]; then
  echo "Detected $PRODUCT setting to Xavier init"
  POLULU=$(/usr/bin/python3 -m auv.utils.devieHelper polulu)
  echo "Found Polulu Servo driver at $POLULU"
  screen -dmS polulu bash -c "bash /home/inspiration/auv/maestro-linux/clearPoluluErrors.sh $POLULU"
  DISTRO="noetic"
fi
if [[ $PRODUCT == *"Nano"* ]]; then
  echo "Detected $PRODUCT setting to Nano init"
  DISTRO="melodic"
fi

OUTPUT=$(/usr/bin/python3 -m auv.utils.deviceHelper)
PORT=`python3 -c "print('''$OUTPUT'''.split()[-1])"`

echo "Found pixhawk on "${PORT}
screen -dmS roscore bash -c "source /opt/ros/$DISTRO/setup.bash ; roscore"
screen -dmS mavros bash -c "source /opt/ros/$DISTRO/setup.bash ; sleep 5 ; roslaunch --wait mavros px4.launch fcu_url:=$PORT"

# Wait for mavros to start then set modprobe for cams
sleep 15
sudo modprobe -r v4l2loopback

# Requires SSHPASS var to be set in ~/.bashrc
screen -dmS cams bash -c "sleep 10 ; sshpass -e /usr/bin/python3 -m auv.device.camsVersatile"
screen -dmS imu bash -c "/usr/bin/python3 -m auv.device.imu.vn100_serial"
screen -dmS dvl bash -c "/usr/bin/python3 -m auv.device.dvl.dvl"
screen -dmS ekfNode bash -c "/usr/bin/python3 -m auv.localization.ekfNode"
screen -dmS modem bash -c "/usr/bin/python3 -m auv.device.modems.ds_modems_node"

if [[ $PRODUCT == *"Xavier"* ]]; then
screen -dmS fog bash -c "/usr/bin/python3 -m auv.device.fog.simple_fog"
screen -dmS maestro_server bash -c "/usr/bin/python3 -m auv.device.maestro.maestro_server"
fi

echo "Done"

