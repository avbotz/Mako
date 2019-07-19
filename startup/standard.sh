# Create TMUX session with needed panes.
tmux new-session -d
tmux split-window -h -p 40
tmux selectp -t 1
tmux split-window -v
tmux selectp -t 3
tmux split-window -v
tmux selectp -t 3
tmux split-window -v
tmux selectp -t 5
tmux split-window -v

# Source workspace in all panes.
tmux selectp -t 1
tmux send-keys "source workspace.sh" C-m 
tmux selectp -t 2
tmux send-keys "source workspace.sh" C-m 
tmux selectp -t 3
tmux send-keys "source workspace.sh" C-m 
tmux selectp -t 4
tmux send-keys "source workspace.sh" C-m 
tmux selectp -t 5
tmux send-keys "source workspace.sh" C-m 
tmux selectp -t 6
tmux send-keys "source workspace.sh" C-m 

# Start standard commands.
tmux selectp -t 1
tmux send-keys "vim -c NERDTree" C-m 
tmux selectp -t 3
tmux send-keys "roscore" C-m 
tmux selectp -t 4
sleep 2
tmux send-keys "rosrun vision acquisition_node" C-m 
tmux selectp -t 2
tmux send-keys "rosrun vision vision_node" C-m 
tmux selectp -t 5
tmux send-keys "rosrun control control_node" C-m 
tmux selectp -t 6
sleep 2
tmux send-keys "rosrun mission restart_node" C-m 

tmux a
