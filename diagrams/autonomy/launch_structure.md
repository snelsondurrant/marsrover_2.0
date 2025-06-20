```mermaid
stateDiagram-v2
  direction LR
  [*] --> BASE
  [*] --> ROVER
  BASE --> base_autonomy.launch.py:rover_bringup
  BASE --> base_launch.xml:ublox_read_2
  ROVER --> rover_autonomy.launch.py:rover_bringup
  ROVER --> rover_startup.launch.py:rover_bringup
  ROVER --> rover_launch.xml:ublox_read_2
  ROVER --> zed_camera.launch.py:zed_wrapper
  base_autonomy.launch.py --> rviz.launch.py:rover_gui
  base_autonomy.launch.py --> mapviz.launch.py:rover_gui
  base_autonomy.launch.py --> autonomy_gui.launch.py:rover_gui
  rover_autonomy.launch.py --> robot_state_publisher.launch.py:rover_description
  rover_autonomy.launch.py --> dual_ekf_navsat.launch.py:rover_localization
  rover_autonomy.launch.py --> aruco_opencv.launch.py:rover_perception
  rover_autonomy.launch.py --> state_machine.launch.py:rover_navigation
  rover_startup.launch.py --> mobility.launch.py:rover_control
  rover_startup.launch.py --> peripherals.launch.py:rover_control
```

--

Created by Nelson Durrant, Jun 2025