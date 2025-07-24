#!/usr/bin/env python3

import rospy
import time
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped

# Liste des objectifs (position + orientation en quaternion)
goals = [
    {'x': 12.2625, 'y': 10.2063, 'z': 0.25483725984376915, 'w': 0.9669839559141192},
    {'x': 8.7610, 'y': 16.5011, 'z': 0.8809651954505093, 'w': 0.47318106936440935},
    {'x': -4.3234, 'y': 8.4929, 'z': -0.9682216616399851, 'w': 0.2500936103364221},
    {'x': -0.6678, 'y': 2.7725, 'z': -0.4922101261514887, 'w': 0.8704764165179523},
    {'x': -4.8942, 'y': -0.1988, 'z': -0.9584888786513258, 'w': 0.28512991688303074},
    {'x': -8.6733, 'y': 6.2899, 'z': 0.8711395357982045, 'w': 0.49103554776542274},
    {'x': -13.7904, 'y': 2.9435, 'z': -0.9633713915207506, 'w': 0.26817077021810687},
    {'x': -10.4422, 'y': -2.9403, 'z': -0.5084958290787575, 'w': 0.8610644527615265}
]

def send_goals():
    pub = rospy.Publisher('/robot/move_base/goal', MoveBaseActionGoal, queue_size=10)
    rospy.init_node('auto_goal_sender', anonymous=True)
    rospy.sleep(1)  # Laisse le temps au publisher d'être prêt

    seq = 1
    for i, goal in enumerate(goals):
        msg = MoveBaseActionGoal()
        msg.header.seq = seq
        msg.header.stamp = rospy.Time.now()
        msg.goal_id.stamp = rospy.Time(0)
        msg.goal_id.id = ""
        msg.goal.target_pose.header.seq = seq
        msg.goal.target_pose.header.stamp = rospy.Time.now()
        msg.goal.target_pose.header.frame_id = "robot_map"

        msg.goal.target_pose.pose.position.x = goal['x']
        msg.goal.target_pose.pose.position.y = goal['y']
        msg.goal.target_pose.pose.position.z = 0.0
        msg.goal.target_pose.pose.orientation.x = 0.0
        msg.goal.target_pose.pose.orientation.y = 0.0
        msg.goal.target_pose.pose.orientation.z = goal['z']
        msg.goal.target_pose.pose.orientation.w = goal['w']

        rospy.loginfo(f"Envoi de l'objectif {i+1}/{len(goals)} : ({goal['x']:.2f}, {goal['y']:.2f})")
        pub.publish(msg)
        seq += 1

        # Attendre un peu avant d'envoyer le suivant
        rospy.sleep(15)  # à ajuster selon ton robot (ou ajouter un feedback d'arrivée)

if __name__ == '__main__':
    try:
        send_goals()
    except rospy.ROSInterruptException:
        pass
        
        
        
        
        
        
        
