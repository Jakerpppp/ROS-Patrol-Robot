#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.srv import MoveToRoom
from second_coursework.msg import BehaviourAction, BehaviourGoal

rospy.init_node('main_node', anonymous=True)

# Starts at A
rospy.wait_for_service("move_to_room")
move_to_room_proxy = rospy.ServiceProxy("move_to_room", MoveToRoom)
response = move_to_room_proxy("A")
rospy.loginfo("Service Call succeeded: Response: %s", response)

# Patrol
patrol_client = actionlib.SimpleActionClient("patrol", BehaviourAction)
patrol_client.wait_for_server()
goal = BehaviourGoal()
goal.number_of_checks = rospy.get_param('~nchecks', 3)
patrol_client.send_goal(goal)
patrol_client.wait_for_result()
result = patrol_client.get_result()
rospy.loginfo("Action Call succeeded: Result: %s", result)
if result:
    print("Rule 1 was broken", result.number_of_times_rules_were_broken[0], "times")
    print("Rule 2 was broken", result.number_of_times_rules_were_broken[1], "times")

# Create the launch file after doing the state machine
