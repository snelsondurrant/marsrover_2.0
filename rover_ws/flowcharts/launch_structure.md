# Created by Nelson Durrant, Jun 2025
# You can edit this code and generate a new PNG file at: https://www.mermaidchart.com/
---
config:
  layout: elk
  theme: neo
  look: classic
---
stateDiagram
  direction LR
  [*] --> base_autonomy.launch.py:rover_bringup
  [*] --> rover_autonomy.launch.py:rover_bringup
  [*] --> rover_startup.launch.py:rover_bringup
  base_autonomy.launch.py --> rviz.launch.py:rover_gui
  base_autonomy.launch.py --> mapviz.launch.py:rover_gui
  base_autonomy.launch.py --> autonomy_gui.launch.py:rover_gui
  rover_autonomy.launch.py --> robot_state_publisher.launch.py:rover_description
  rover_autonomy.launch.py --> dual_ekf_navsat.launch.py:rover_localization
  rover_autonomy.launch.py --> aruco_opencv.launch.py:rover_perception
  rover_autonomy.launch.py --> state_machine.launch.py:rover_navigation
  rover_startup.launch.py --> mobility.launch.py:rover_control
  rover_startup.launch.py --> peripherals.launch.py:rover_control