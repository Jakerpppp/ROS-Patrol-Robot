#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.msg import BehaviourAction, BehaviourGoal, BehaviourResult, BehaviourFeedback
import smach
from smach_ros import ServiceState
from states.navigateToSpecificState import NavigateToAState, NavigateToBState, NavigateToDState
from states.yoloSpecificState import YOLOStateA, YOLOStateB, YOLOStateD
from second_coursework.srv import MoveToRoom


@smach.cb_interface(input_keys=['current_count', 'goal_count'], output_keys=['current_count'],
                    outcomes=['continue_cycle', 'end_cycle'])
def process_cycle_cb(userdata):
    if userdata.current_count == userdata.goal_count:
        return 'end_cycle'
    else:
        userdata.current_count += 1
        return 'continue_cycle'


@smach.cb_interface(input_keys=['feedback'], output_keys=['feedback'],
                    outcomes=['succeeded'])
def process_feedback_cb(userdata):
    for data in userdata.feedback:
        feedback_data = BehaviourFeedback()
        feedback_data.rule_broken = data[1]
        feedback_data.position_of_detection = data[0]
        action_server.publish_feedback(feedback_data)
        print("Published Feedback", feedback_data)
    userdata.feedback.clear()
    return 'succeeded'


# Child Terminations for Concurrent State Machine: Stops all other States once finished Navigating
def child_term_A_cb(outcome_map):
    if outcome_map['NAVIGATE_AROUND_A']:
        return True
    else:
        return False


def child_term_B_cb(outcome_map):
    if outcome_map['NAVIGATE_AROUND_B']:
        return True
    else:
        return False


def child_term_D_cb(outcome_map):
    if outcome_map['NAVIGATE_AROUND_D']:
        return True
    else:
        return False


def start_patrol(goal: BehaviourGoal):
    result = BehaviourResult()

    # State Machine that uses CBStates, ServiceStates and Concurrency
    '''Explanation of State Machine:
    - Separate States for Navigation and Move and YOLO for each Room A,B,D 
    - Concurrent States of YOLO and Navigation to allow for YOLO Processing
    - Feedback info is collected and then processed and published after each navigation'''

    sm = smach.StateMachine(outcomes=["Finished_Patrol", 'preempted', 'aborted'])

    with sm:
        smach.StateMachine.add('PROCESS_CYCLE', smach.CBState(process_cycle_cb),
                               transitions={'continue_cycle': 'CONCURRENT_NAV_OF_A_AND_YOLO',
                                            'end_cycle': 'Finished_Patrol'},
                               remapping={'current_count': 'current_count', 'goal_count': 'goal_count'})

        # Concurrent State Machine for YOLO and Navigation - So YOLO can detect while Navigating for Room A
        ccA = smach.Concurrence(outcomes=['Finished_Navigating'],
                                default_outcome='Finished_Navigating',
                                input_keys=['result', 'feedback'],
                                output_keys=['next_room', 'result', 'feedback'],
                                child_termination_cb=child_term_A_cb)
        with ccA:
            smach.Concurrence.add('NAVIGATE_AROUND_A', NavigateToAState())

            smach.Concurrence.add('YOLO_ROOM_A', YOLOStateA(),
                                  remapping={'result': 'result', 'feedback': 'feedback'})

        smach.StateMachine.add('CONCURRENT_NAV_OF_A_AND_YOLO', ccA,
                               transitions={'Finished_Navigating': 'PROCESS_A_RULES_BROKEN'})

        smach.StateMachine.add('PROCESS_A_RULES_BROKEN', smach.CBState(process_feedback_cb),
                               transitions={'succeeded': 'MOVE_TO_B'},
                               remapping={'feedback': 'feedback'})



        smach.StateMachine.add('MOVE_TO_B', ServiceState('move_to_room', MoveToRoom, request_slots=['room_name']),
                               transitions={'succeeded': 'CONCURRENT_NAV_OF_B_AND_YOLO'},
                               remapping={'room_name': 'next_room'})

        # Concurrent State Machine for YOLO and Navigation - So YOLO can detect while Navigating for Room B
        ccB = smach.Concurrence(outcomes=['Finished_Navigating'],
                                default_outcome='Finished_Navigating',
                                input_keys=['result', 'feedback'],
                                output_keys=['next_room', 'result', 'feedback'],
                                child_termination_cb=child_term_B_cb)
        with ccB:
            smach.Concurrence.add('NAVIGATE_AROUND_B', NavigateToBState())

            smach.Concurrence.add('YOLO_ROOM_B', YOLOStateB(),
                                  remapping={'result': 'result', 'feedback': 'feedback'})

        smach.StateMachine.add('CONCURRENT_NAV_OF_B_AND_YOLO', ccB,
                               transitions={'Finished_Navigating': 'PROCESS_B_RULES_BROKEN'})

        smach.StateMachine.add('PROCESS_B_RULES_BROKEN', smach.CBState(process_feedback_cb),
                               transitions={'succeeded': 'MOVE_TO_D'},
                               remapping={'feedback': 'feedback'})



        # Room D is Visited after visiting Room A then Room B
        smach.StateMachine.add('MOVE_TO_D', ServiceState('move_to_room', MoveToRoom, request_slots=['room_name']),
                               transitions={'succeeded': 'CONCURRENT_NAV_OF_D_AND_YOLO'},
                               remapping={'room_name': 'next_room'})

        # Concurrent State Machine for YOLO and Navigation - So YOLO can detect while Navigating for Room D
        ccD = smach.Concurrence(outcomes=['Finished_Navigating'],
                                default_outcome='Finished_Navigating',
                                input_keys=['result', 'feedback'],
                                output_keys=['next_room', 'result', 'feedback'],
                                child_termination_cb=child_term_D_cb)
        with ccD:
            smach.Concurrence.add('NAVIGATE_AROUND_D', NavigateToDState())

            smach.Concurrence.add('YOLO_ROOM_D', YOLOStateD(),
                                  remapping={'result': 'result', 'feedback': 'feedback'})

        smach.StateMachine.add('CONCURRENT_NAV_OF_D_AND_YOLO', ccD,
                               transitions={'Finished_Navigating': 'PROCESS_D_RULES_BROKEN'})

        smach.StateMachine.add('PROCESS_D_RULES_BROKEN', smach.CBState(process_feedback_cb),
                               transitions={'succeeded': 'MOVE_TO_A'},
                               remapping={'feedback': 'feedback'})



        smach.StateMachine.add('MOVE_TO_A', ServiceState('move_to_room', MoveToRoom, request_slots=['room_name']),
                               transitions={'succeeded': 'PROCESS_CYCLE'},
                               remapping={'room_name': 'next_room'}, )



    # Set the Initial Data
    sm.userdata['current_count'] = 0  # The initial counts of checks of A and B
    sm.userdata['goal_count'] = goal.number_of_checks  # The goal number of checks
    sm.userdata['result'] = [0, 0]  # Initialise Results List
    sm.userdata['feedback'] = []  # Feedback tp be published after navigation
    sm.execute()

    result.number_of_times_rules_were_broken = sm.userdata['result']
    action_server.set_succeeded(result)


rospy.init_node('behaviour_node', anonymous=True)
action_server = actionlib.SimpleActionServer("patrol", BehaviourAction, start_patrol)
rospy.spin()
