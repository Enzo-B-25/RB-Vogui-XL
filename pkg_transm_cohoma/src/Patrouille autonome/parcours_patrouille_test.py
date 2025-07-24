import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import math

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

def distance_remaining(feedback, goal_x, goal_y):
    pos = feedback.base_position.pose.position
    return math.sqrt((goal_x - pos.x)**2 + (goal_y - pos.y)**2)

def send_goal_and_wait(client, goal_data, timeout=60.0):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_data['x']
    goal.target_pose.pose.position.y = goal_data['y']
    goal.target_pose.pose.orientation.z = goal_data['z']
    goal.target_pose.pose.orientation.w = goal_data['w']

    def done_cb(status, result):
        if not isinstance(status, int):
            rospy.logerr("❌ Type inattendu pour le status.")
            return

        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("🎯 Objectif atteint !")
        elif status == GoalStatus.ABORTED:
            rospy.logwarn("⚠️ Objectif échoué (ABORTED).")
        elif status == GoalStatus.REJECTED:
            rospy.logwarn("🚫 Objectif rejeté (REJECTED).")
        elif status == GoalStatus.LOST:
            rospy.logerr("❓ Objectif perdu (LOST).")
        else:
            rospy.loginfo(f"🔄 Objectif terminé avec le statut {status}")

    def feedback_cb(feedback):
        dist = distance_remaining(feedback, goal_data['x'], goal_data['y'])
        rospy.loginfo_throttle(2.0, f"📍 Distance restante : {dist:.2f} m")

    rospy.loginfo(f"🚀 Envoi objectif à x={goal_data['x']}, y={goal_data['y']}")
    client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)

    finished = client.wait_for_result(rospy.Duration(timeout))
    if not finished:
        rospy.logerr("⏱️ Timeout ! Annulation de l’objectif.")
        client.cancel_goal()
        return False

    state = client.get_state()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("✅ Objectif terminé avec succès (via get_state).")
        return True
    else:
        rospy.logwarn(f"⚠️ Objectif terminé avec le statut : {state}")
        return False

if __name__ == "__main__":
    rospy.init_node("navigation_multi_goals")

    client = actionlib.SimpleActionClient('robot/move_base', MoveBaseAction)
    rospy.loginfo("⏳ Connexion à move_base...")
    client.wait_for_server()
    rospy.loginfo("✅ Connecté à move_base.")

    for i, goal in enumerate(goals):
        rospy.loginfo(f"\n➡️ Objectif {i+1}/{len(goals)}")
        success = send_goal_and_wait(client, goal, timeout=90.0)

        if not success:
            rospy.logwarn("🛑 Arrêt de la séquence à cause d’un échec.")
            break

    rospy.loginfo("🏁 Séquence terminée.")
