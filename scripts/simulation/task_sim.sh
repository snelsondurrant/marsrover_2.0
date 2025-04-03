#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Runs the autonomy task in the simulation environment

legs_to_run='{legs: ["gps1", "aruco1", "mallet"]}'

# ANSI color codes
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

action_pid=0

trap cleanup INT
cleanup() {
  if [[ $action_pid -ne 0 ]]; then
    ros2 service call /exec_autonomy_task/_action/cancel_goal action_msgs/srv/CancelGoal "{}" > /dev/null 2>&1
    kill $action_pid 2>/dev/null
    wait $action_pid 2>/dev/null
  fi
  exit 1
}

source ~/rover_ws/install/setup.bash
ros2 action send_goal --feedback exec_autonomy_task rover_interfaces/action/RunTask "$legs_to_run" 2>/dev/null | while IFS= read -r line; do
  if [[ "$line" == *'ERROR'* ]] || [[ "$line" == *'FATAL'* ]]; then
    echo -e "${RED}$line${NC}"
  elif [[ "$line" == *'WARN'* ]]; then
    echo -e "${YELLOW}$line${NC}"
  elif [[ "$line" == *'SUCCESS'* ]]; then
    echo -e "${GREEN}$line${NC}"
  else
    echo "$line"
  fi
done &

action_pid=$!
wait $action_pid
