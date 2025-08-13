#!/bin/bash

cd $(dirname $(realpath $0))
SESSION_NAME="multi_window"

# Start new tmux session with first window
tmux new-session -d -s $SESSION_NAME -n "win1"

# Function to split window into 9 panes and run commands
split_into_n() {
    local target=$1   # e.g. "session:window"
    local count=$2    # total number of panes you want

    if (( count < 1 )); then
        echo "Error: count must be >= 1"
        return 1
    fi

    tmux select-layout -t "$target" tiled
    for ((i = 1; i < count; i++)); do
        tmux split-window -t "$target"
        tmux select-layout -t "$target" tiled
    done
}

# Create and arrange first window
split_into_n "${SESSION_NAME}:win1" 9

# Now run commands in each of the 9 panes
tmux send-keys -t "win1".0 "roscore" C-m
sleep 10

tmux send-keys -t "win1".1 "roslaunch mavros px4.launch" C-m
sleep 10

tmux send-keys -t "win1".2 "python3 -m auv.device.maestro.maestro_server" C-m
tmux send-keys -t "win1".3 "python3 -m auv.device.modems.ds_modems_node" C-m
tmux send-keys -t "win1".4 "python3 -m auv.device.imu.vn100_serial" C-m
tmux send-keys -t "win1".5 "python3 -m auv.device.dvl.dvl" C-m
tmux send-keys -t "win1".6 "python3 -m auv.device.fog.simple_fog" C-m
tmux send-keys -t "win1".7 "python3 -m auv.localization.ekfNode" C-m

# Create second and third windows
tmux new-window -t $SESSION_NAME -n "win2" 
split_into_n "${SESSION_NAME}:win2" 4
tmux send-keys -t "win2".0 "sshpass -e sudo modprobe -r v4l2loopback; sshpass -e python3 -m auv.device.camsVersatile" C-m
tmux send-keys -t "win2".1 "cd ../rtsp/" C-m
tmux send-keys -t "win2".2 "cd ../companion/scripts/" C-m
tmux send-keys -t "win2".3 "" C-m

tmux new-window -t $SESSION_NAME -n "win3"
split_into_n "${SESSION_NAME}:win3" 6
tmux send-keys -t "win3".0 "rostopic echo /mavros/state" C-m
tmux send-keys -t "win3".1 "rostopic echo /auv/state/pose" C-m
tmux send-keys -t "win3".2 "python3 -m auv.utils.fly STABILIZE" C-m
tmux send-keys -t "win3".3 "disarm" C-m
tmux send-keys -t "win3".4 "echo 'Run your mission here'" C-m
tmux send-keys -t "win3".5 "./recalibrate.sh"


# Attach to session
tmux attach -t $SESSION_NAME