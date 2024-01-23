import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import smach
import random
import math

'''Parent Class for the specific navigation to each room - Each Room Navigation Inherits this class with specific 
details about the room '''


class NavigateState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['next_room'])

    # ud is userdata
    def execute(self, ud):
        self.set_next_room(ud)
        move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        move_base_client.wait_for_server()

        start_time = rospy.get_rostime()
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.header.frame_id = "map"

        while (rospy.get_rostime() - start_time).to_sec() < self.duration:
            goal_x = random.uniform(self.min_x, self.max_x)
            goal_y = random.uniform(self.min_y, self.max_y)

            goal.target_pose.pose.position.x = goal_x
            goal.target_pose.pose.position.y = goal_y
            goal.target_pose.pose.orientation.z = math.sin(-math.pi / 4)
            goal.target_pose.pose.orientation.w = math.cos(-math.pi / 4)

            move_base_client.send_goal(goal)
            move_base_client.wait_for_result(timeout=rospy.Duration(2))

        return 'succeeded'

    def set_next_room(self, ud):
        pass




