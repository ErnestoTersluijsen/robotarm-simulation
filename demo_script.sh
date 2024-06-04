#!/bin/bash
colcon build

. install/setup.sh

ros2 launch robotarm_simulation robotarm_simulation.launch.py &

sleep 3

ros2 topic pub --once /commands std_msgs/msg/String '{data: "#5P2500T500\r"}'
ros2 topic pub --once /commands std_msgs/msg/String '{data: "#1P1050T3000\r"}'

sleep 3.5

ros2 topic pub --once /commands std_msgs/msg/String '{data: "#5P1500T1000\r"}'

sleep 1.5

ros2 topic pub --once /commands std_msgs/msg/String '{data: "#1P1500T4000\r"}'

sleep 4.5

ros2 topic pub --once /commands std_msgs/msg/String '{data: "#5P2500T500\r"}'
