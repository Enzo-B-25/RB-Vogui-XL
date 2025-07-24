#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Liste des objectifs
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
    rospy.init_node('auto_goal_sender_action_client')
    
    rospy.loginfo("Connexion à /robot/move_base...")
    client = actionlib.SimpleActionClient('/robot/move_base', MoveBaseAction)
    
    if not client.wait_for_server(rospy.Duration(10)):
        rospy.logerr("Impossible de se connecter à /robot/move_base")
        return

    rospy.loginfo("Connecté à move_base")

    for i, goal in enumerate(goals):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id = "robot_map"
        move_goal.target_pose.header.stamp = rospy.Time.now()
        move_goal.target_pose.pose.position.x = goal['x']
        move_goal.target_pose.pose.position.y = goal['y']
        move_goal.target_pose.pose.orientation.z = goal['z']
        move_goal.target_pose.pose.orientation.w = goal['w']

        rospy.loginfo(f"Envoi de l'objectif {i+1}/{len(goals)} : ({goal['x']:.2f}, {goal['y']:.2f})")
        client.send_goal(move_goal)

        # Attendre que l’objectif soit atteint
        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo(f"Objectif {i+1} atteint.")

if __name__ == '__main__':
    try:
        send_goals()
    except rospy.ROSInterruptException:
        pass
