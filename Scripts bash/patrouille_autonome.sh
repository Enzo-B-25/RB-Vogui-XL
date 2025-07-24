#!/bin/bash

# Publier la position initiale (2D Pose Estimate)
rostopic pub -1 /robot/initialpose geometry_msgs/PoseWithCovarianceStamped -- "
header:
  frame_id: 'robot_map'
pose:
  pose:
    position:
      x: -9.728217124938965
      y: -4.464015007019043
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.3078990276548005
      w: 0.9514190395242406
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]"


sleep 2

# Lancer le script Python de patrouille (modifie 'ton_package' par le nom réel de ton package)
rosrun pkg_transm_cohoma parcours_patrouille_test.py & PID_PATROUILLE=$!

# Attente pour pouvoir faire Ctrl+C
trap "kill $PID_PATROUILLE" SIGINT
wait

