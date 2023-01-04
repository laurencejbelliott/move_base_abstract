#!/usr/bin/python
from math import sqrt
import rospy
# import actionlib
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState

# TODO:
#  - Make this an actionserver, so it can fully replace move_base
#  - Parameterise top speed, simulator (gazebo / stage_ros), robot model name

top_speed = 1.5  # m/s - Thorvald's max. speed
gazebo_pose_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
ground_truth_pose = Pose()


def ground_truth_pose_cb(msg):
    global ground_truth_pose
    ground_truth_pose = msg.pose.pose


def goal_pose_cb(msg):
    goal_pose = msg
    model_state = ModelState()
    model_state.model_name = 'thorvald_001'
    model_state.pose = goal_pose
    rospy.loginfo("Goal pose received: %s", goal_pose)

    # Calculate the distance to the goal
    distance = sqrt(((goal_pose.position.x - ground_truth_pose.position.x)**2) +
                    ((goal_pose.position.y - ground_truth_pose.position.y)**2))
    time_to_goal = distance / top_speed
    rospy.loginfo("Distance to goal: %s", distance)
    rospy.loginfo("Robot will reach the goal in %s seconds", time_to_goal)
    rospy.sleep(time_to_goal)

    gazebo_pose_pub.publish(model_state)
    rospy.loginfo("Goal pose published to gazebo")


def move_base_goal_cb(msg):
    goal_pose = msg.goal.target_pose.pose
    goal_pose_cb(goal_pose)


if __name__ == '__main__':
    # rospy.Subscriber('/move_base_abstract/goal', Pose, goal_pose_cb)
    rospy.Subscriber('/thorvald_001/move_base/goal', MoveBaseActionGoal, move_base_goal_cb)
    rospy.Subscriber('/thorvald_001/amcl_pose', PoseWithCovarianceStamped, ground_truth_pose_cb)
    rospy.init_node('move_base_abstract_actionserver')
    rospy.spin()
