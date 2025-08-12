#!/bin/bash

PRODUCT=$(sudo lshw -json | jq '.product') || PRODUCT=$(sudo lshw -json | jq '.[].product')

# cd into absolute path of robosub repo
AUV_PATH=$(dirname $(dirname $(realpath $0)))
echo $AUV_PATH
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

echo "Found pixhawk on "${OUTPUT}
screen -dmS roscore bash -c "source /opt/ros/$DISTRO/setup.bash ; roscore"

# mavros crashes, need to fix
screen -dmS mavros bash -c "source /opt/ros/$DISTRO/setup.bash ; sleep 5 ; roslaunch --wait mavros px4.launch fcu_url:=$OUTPUT"

screen -dmS cams bash -c "sleep 10 ; /usr/bin/python3 -m auv.device.camsVersatile"
screen -dmS imu bash -c "/usr/bin/python3 -m auv.device.imu.vn100_serial"
screen -dmS dvl bash -c "/usr/bin/python3 -m auv.device.dvl.dvl"
screen -dmS ekfNode bash -c "/usr/bin/python3 -m auv.localization.ekfNode"
screen -dmS modem bash -c "/usr/bin/python3 -m auv.device.modems.ds_modems_node"

if [[ $PRODUCT == *"Xavier"* ]]; then
screen -dmS fog bash -c "/usr/bin/python3 -m auv.device.fog.simple_fog"
screen -dmS maestro_server bash -c "/usr/bin/python3 -m auv.device.maestro.maestro_server"
fi

echo "Done"

