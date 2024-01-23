#!/usr/bin/env python3
import math
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from second_coursework.srv import MoveToRoom, MoveToRoomRequest, MoveToRoomResponse
import actionlib




def move_to_room(request: MoveToRoomRequest):
    response = MoveToRoomResponse()
    response.destination_reached = False

    if request.room_name not in ("A", "B", "D"):
        return response

    move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base_client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.get_rostime()
    goal.target_pose.header.frame_id = "map"

    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = math.sin(-math.pi/4)
    goal.target_pose.pose.orientation.w = math.cos(-math.pi / 4)

    if request.room_name == "A":
        goal.target_pose.pose.position.x = 2
        goal.target_pose.pose.position.y = 8

    if request.room_name == "B":
        goal.target_pose.pose.position.x = 6
        goal.target_pose.pose.position.y = 8

    if request.room_name == "D":
        goal.target_pose.pose.position.x = 2
        goal.target_pose.pose.position.y = 3

    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()
    status = move_base_client.get_state()

    if status == actionlib.GoalStatus.SUCCEEDED:
        response.destination_reached = True

    return response


rospy.init_node('move_robot_node', anonymous=True)
room_service = rospy.Service("move_to_room", MoveToRoom, move_to_room)
rospy.spin()

