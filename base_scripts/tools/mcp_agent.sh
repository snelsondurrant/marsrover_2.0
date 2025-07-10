#!/bin/bash
# Created by Nelson Durrant, July 2025
#
# EXPERIMENTAL! See 'scripts/simulation/mcp_server.py' for more details.
# Super-simple AI agent to test the perceive -> decide -> act loop.

echo "Starting MCP Agent..."
echo ""

response=$(gemini -m gemini-2.5-flash -p "Generate a state report for the rover using the MCP \
tools. Include the rovers current GPS location, odometry data (position, orientation, and velocity), \
and whether it is moving or not. Small, negligable movements do not count as the rover moving. \
Simplify your output and don't include any bolding or fancy formatting.")
echo "$response"
echo ""

gemini -m gemini-2.5-flash -p "Here is the rovers state report: $response. \
If it is not moving, please create a new gps waypoint task with a location 2 meters north \
of the rovers current position. Report how the task went in your response. Simplify \
your output and don't include any bolding or fancy formatting."
echo ""

echo "MCP Agent finished."
