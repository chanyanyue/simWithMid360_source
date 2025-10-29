#!/bin/bash

# 创建会话和第一个窗口
tmux new-session -d -s ros_session -n main_nodes

# Pane 0: roscore
tmux send-keys -t ros_session:0 'roscore' C-m

# Pane 1: sim.launch 启动仿真
tmux split-window -h -t ros_session:0
tmux send-keys -t ros_session:0.1 'sleep 3; roslaunch tutorial_gazebo sim.launch' C-m

# Pane 2: gazebo_sim.launch 点云变换与定位映射
tmux split-window -v -t ros_session:0.1
tmux send-keys -t ros_session:0.2 'sleep 5; source ~/sim_ws/devel/setup.bash && roslaunch tutorial_gazebo gazebo_sim.launch' C-m

# 整理第一个窗口布局
tmux select-layout -t ros_session:0 tiled
# --------------------
# 第二窗口（监控和任务）
# --------------------
tmux new-window -t ros_session:1 -n monitors_mission

# Pane 0: /mavros/local_position/pose 定位监控
tmux send-keys -t ros_session:1 'sleep 6; rostopic echo /mavros/local_position/pose' C-m

# 整理第三个窗口布局
tmux select-layout -t ros_session:1 tiled

# 附加到会话并显示第一个窗口
tmux select-window -t ros_session:0
tmux attach-session -t ros_session:1