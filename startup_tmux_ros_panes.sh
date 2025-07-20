#!/bin/bash

SESSION_NAME="ros_pane_setup"
DISTRO=${ROS_DISTRO}
# Start a new tmux session with one window
tmux new-session -d -s $SESSION_NAME -n "main"

source /opt/ros/noetic/setup.bash
# Split horizontally to create Pane 1 (top-right)
tmux split-window -h -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME:0.1
tmux split-window -v -t $SESSION_NAME:0.4
tmux split-window -v -t $SESSION_NAME:0.4

# Send echo command to each pane
tmux send-keys -t $SESSION_NAME:0.0 'roscore' C-m
tmux send-keys -t $SESSION_NAME:0.1 'source /opt/ros/$ROS_DISTRO/setup.bash' C-m
sleep 2
tmux send-keys -t $SESSION_NAME:0.1 'roslaunch mavros px4.launch' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:0.2 'python3 -m auv.device.imu.vn100_serial' C-m
tmux send-keys -t $SESSION_NAME:0.3 'python3 -m auv.device.dvl.dvl' C-m
sleep 2
tmux send-keys -t $SESSION_NAME:0.4 'python3 -m auv.localization.ekfNode' C-m
tmux send-keys -t $SESSION_NAME:0.5 'echo "Pane 5"' C-m
tmux send-keys -t $SESSION_NAME:0.6 'echo "Pane 6"' C-m

sleep 5
# Attach to session
tmux attach-session -t $SESSION_NAME
