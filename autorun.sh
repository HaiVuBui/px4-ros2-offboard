#!/bin/bash

# Define commands for each pane
cmd1='cd ~/microros_ws;
        source install/local_setup.bash;
        export ROS_DOMAIN_ID=0;
        export PYTHONOPTIMIZE=1;
        ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0'
cmd2='cd ~/PX4-Autopilot;
        export ROS_DOMAIN_ID=0;
        export PYTHONOPTIMIZE=1;
        HEADLESS=1 make px4_sitl gazebo'
cmd3='cd ~/Downloads;
        ./QGroundControl.AppImage'
cmd4='cd px4_offboard/;
        source install/setup.bash;
        ros2 launch px4_offboard offboard_position_control.launch.py'

# Create a new tmux session named 'mysession' and detach
tmux new-session -d -s mysession

# Rename the default window (optional)
tmux rename-window -t mysession:0 'MainWindow'

# Split the window into 2 horizontal panes
tmux split-window -h

# Split the right pane into 2 vertical panes
tmux split-window -v -t mysession:0.1

# Select the left pane
tmux select-pane -t mysession:0.0

# Split the left pane into 2 vertical panes
tmux split-window -v -t mysession:0.0

# Send commands to each pane
tmux send-keys -t mysession:0.0 "$cmd1" C-m
tmux send-keys -t mysession:0.1 "$cmd2" C-m
#tmux send-keys -t mysession:0.2 "$cmd3" C-m
tmux send-keys -t mysession:0.3 "$cmd4" C-m

# Attach to the session
tmux attach-session -t mysession
