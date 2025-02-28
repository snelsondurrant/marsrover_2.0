#! #!/bin/bash
# Created by Nelson Durrant, Feb 2025

source ~/ros2_ws/install/setup.bash
ros2 launch nav2_autonomy state_machine_sim.launch.py use_rviz:=True use_mapviz:=True
