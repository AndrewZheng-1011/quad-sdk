#!/bin/bash

# Notes:
# Runs ICRA world with Quad-SDK with predefined waypoints/intermediate goals

PID=$!
start_time=$SECONDS
echo "Start time: $start_time"

roslaunch quad_utils quad_plan.launch
sleep 1

kill $PID

# roslaunch quad_utils quad_plan.launch goal_plan:=2

# sleep 10

# roslaunch quad_utils quad_plan.launch goal_plan:=3
