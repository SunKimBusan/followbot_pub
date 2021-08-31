tmux kill-session
sleep 1
tmux new-session -d
tmux set -g mouse on
tmux split-window -h
tmux select-pane -R
tmux split-window -h
tmux select-pane -R
tmux split-window -h
tmux select-pane -R
tmux split-window -h
tmux select-pane -R
tmux select-layout even-horizontal

tmux select-pane -t 0
tmux send "source ~/.bashrc" C-m
tmux send "roscd followbot_mechanum" C-m
tmux send "roslaunch followbot_mechanum followbot_mechanum_7.launch" C-m
sleep 1
tmux select-pane -t 1
tmux send "source ~/.bashrc" C-m
tmux send "roscd followbot_mechanum" C-m
tmux send "rosrun followbot_mechanum face_check.py" C-m
sleep 1
tmux select-pane -t 2
tmux send "source ~/.bashrc" C-m
tmux send "roscd followbot_mechanum" C-m
tmux send "rosrun followbot_mechanum clothes_pattern_11.py" C-m
sleep 1
tmux select-pane -t 3
tmux send "source ~/.bashrc" C-m
tmux send "roscd followbot_mechanum" C-m
#tmux send "rosrun followbot_mechanum controller_ver15.py" C-m
sleep 1
tmux select-pane -t 4
tmux send "source ~/.bashrc" C-m
tmux send "roscd followbot_mechanum" C-m
#tmux send

tmux attach-session -d
