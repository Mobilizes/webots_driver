#!/usr/bin/sh
tmux new -s 1 -d -c /home/mob/ros2-ws -n tachimawari
tmux new-window -d -n kansei -t 1: -c /home/mob/ros2-ws
tmux new-window -d -n akushon -t 1: -c /home/mob/ros2-ws
tmux new-window -d -n aruku -t 1: -c /home/mob/ros2-ws
tmux new-window -d -n ichiro-app -t 1: -c /home/mob/ros2-ws/src/ichiro-app/
tmux new-window -d -n envoy -t 1: -c /home/mob/ros2-ws/configuration/hiro/envoy/
tmux new-window -d -n webots -t 1: -c /home/mob/ros2-ws
tmux new-window -d -n soccer -t 1: -c /home/mob/ros2-ws
tmux new-window -d -n publish-button -t 1: -c /home/mob/ros2-ws

# tmux send-keys -t 1:2 C-z 'ros2 run kansei main --path /home/mob/ros2-ws/configuration/hiro/imu/ --type orientation ' Enter
tmux send-keys -t 1:3 C-z 'ros2 run akushon main ~/ros2-ws/configuration/hiro/action/' Enter

tmux send-keys -t 1:4 C-z 'ros2 run aruku main ~/ros2-ws/configuration/hiro/walking/' Enter
tmux send-keys -t 1:5 C-z 'npm run dev' Enter
tmux send-keys -t 1:6 C-z 'envoy -c ./ichiro.yaml' Enter

tmux send-keys -t 1:7 C-z 'ros2 launch webots_driver robot_launch.py' Enter
sleep 1
tmux send-keys -t 1:8 C-z "ros2 run soccer soccer_run ~/ros2-ws/configuration/hiro/soccer/ test $1" Enter
sleep 3

tmux send-keys -t 1:1 C-z 'ros2 launch tachimawari tachimawari_launch.py' Enter

# tmux send-keys -t 1:9 C-z 'ros2 topic pub control/status tachimawari_interfaces/msg/Status "{button: 2, led_panel: 255}" -1 && ros2 topic pub control/status tachimawari_interfaces/msg/Status "{button: 0, led_panel: 255}" -1' Enter
# tmux send-keys -t 1:9 C-z 'ros2 topic pub control/status tachimawari_interfaces/msg/Status "{button: 1, led_panel: 255}" -1 && ros2 topic pub control/status tachimawari_interfaces/msg/Status "{button: 0, led_panel: 255}" -1'

tmux send-keys -t 1:9 C-z 'ros2 topic pub control/status tachimawari_interfaces/msg/Status "{button: 2, led_panel: 255}" -1' Enter
tmux send-keys -t 1:9 C-z 'ros2 topic pub control/status tachimawari_interfaces/msg/Status "{button: 1, led_panel: 255}" -1'

tmux bind -n C-q kill-session
tmux attach -t 1:8
