# Created by Nelson Durrant, Jun 2025
# You can edit this code and generate a new PNG file at: https://www.mermaidchart.com/
---
config:
  layout: elk
  theme: neo
  look: classic
---
stateDiagram
  direction TB
  [*] --> Autonomy_GUI
  Autonomy_GUI --> State_Machine:Destination WP
  State_Machine --> Autonomy_GUI:Progress Report
  State_Machine --> Nav2:Planned Path WPs
  State_Machine --> Path_Planner:Destination WP
  Path_Planner --> State_Machine:Planned Path WPs
  Nav2 --> Gazebo:Drive Commands
  Gazebo --> EKF:IMU Data
  Gazebo --> EKF:Wheel Odometry
  Gazebo --> EKF:GPS Data
  Gazebo --> Nav2:3D Point Clouds
  Gazebo --> Aruco_Detector:Camera Images
  Gazebo --> State_Machine:Detected Objects
  Aruco_Detector --> State_Machine:Detected Tags
  EKF --> State_Machine:Position Estimate
  EKF --> Autonomy_GUI:Position Estimate
  EKF --> Nav2:Position Estimate