#!/bin/bash

# Lancer le noeud joystick
rosrun control_ur_with_joy ur_joy_contro_origine.py &
PID_JOY=$!
sleep 2

# Lancer RViz
rviz -d ~/.rviz/demo_VOGUI.rviz &
PID_RVIZ=$!
sleep 5  # attendre que AMCL soit lancé si besoin

# Publier la pose initiale
rostopic pub -1 /robot/initialpose geometry_msgs/PoseWithCovarianceStamped "
header:
  frame_id: 'robot_map'
pose:
  pose:
    position:
      x: -0.7353358268737793
      y: -1.0378599166870117
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.9006284522007307
      w: 0.4345899113952326
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]"

# Attente pour pouvoir faire Ctrl+C
trap "kill $PID_JOY $PID_RVIZ" SIGINT
wait


