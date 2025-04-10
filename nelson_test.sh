# Quick script to set up the rover for testing

cd ~/Autonomy-ROS2 && bash compose.sh down
cd ~/marsrover_2.0 && bash compose.sh

ssh -t marsrover@192.168.1.120 "cd ~/Autonomy-ROS2 && bash compose.sh down"
ssh -t marsrover@192.168.1.120 "cd ~/marsrover_2.0 && bash compose.sh"

ssh marsrover@192.168.1.120
