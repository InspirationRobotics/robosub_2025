#!/bin/bash

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
for pane_id in {0..8}; do
    case $pane_id in
        0) tmux send-keys -t "win1".${pane_id} "roscore" C-m ;;
        1) tmux send-keys -t "win1".${pane_id} "roslaunch mavros px4.launch" C-m ;;
        2) tmux send-keys -t "win1".${pane_id} "python3 -m auv.device.maestro.maestro_server" C-m ;; 
        3) tmux send-keys -t "win1".${pane_id} "python3 -m auv.device.modems.ds_modems_node" C-m ;;
        4) tmux send-keys -t "win1".${pane_id} "python3 -m auv.device.imu.vn100_serial" C-m ;;
        5) tmux send-keys -t "win1".${pane_id} "python3 -m auv.device.dvl.dvl" C-m ;;
        6) tmux send-keys -t "win1".${pane_id} "python3 -m auv.device.fog.simple_fog" C-m ;;
        7) tmux send-keys -t "win1".${pane_id} "python3 -m auv.localization.ekfNode" C-m ;;
        8) tmux send-keys -t "win1".${pane_id} "echo 'Pane 8: placeholder'" C-m ;;
    esac
done
# Create second and third windows
tmux new-window -t $SESSION_NAME -n "win2" 4
split_into_n "${SESSION_NAME}:win2"
for pane_id in {0..6}; do
    case $pane_id in
        0) tmux send-keys -t "win2".${pane_id} "python3 -m auv.device.camsVersatile" C-m ;;
        1) tmux send-keys -t "win2".${pane_id} "cd ../rtsp/" C-m ;;
        2) tmux send-keys -t "win2".${pane_id} "cd ../companion/script/" C-m ;; 
        3) tmux send-keys -t "win2".${pane_id} "" C-m ;;
    esac
done
tmux new-window -t $SESSION_NAME -n "win3" 5
split_into_n "${SESSION_NAME}:win3"
for pane_id in {0..6}; do
    case $pane_id in
        0) tmux send-keys -t "win3".${pane_id} "rostopic echo /mavros/state" C-m ;;
        1) tmux send-keys -t "win3".${pane_id} "rostopic echo /auv/state/pose" C-m ;;
        2) tmux send-keys -t "win3".${pane_id} "python3 -m auv.utils.fly STABILIZE" C-m ;; 
        3) tmux send-keys -t "win3".${pane_id} "disarm" C-m ;;
        4) tmux send-keys -t "win3".${pane_id} "echo 'Run your mission here'" C-m ;;
    esac
done


# Attach to session
tmux attach -t $SESSION_NAME