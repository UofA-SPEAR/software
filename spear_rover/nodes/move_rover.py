#!/usr/bin/python
import actionlib
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach import State, StateMachine, Sequence
from spear_msgs.msg import ArmAngles
from std_msgs.msg import String, Header
from time import sleep

class Error(State):
    def __init__(self):
        State.__init__(self, outcomes=['done', 'seq'],
                            input_keys=['error_msg'], 
                            output_keys=['next_state'])
    def execute(self, userdata):
        # TO DO
        return 'done'

class ProcessUserData(State):
    def __init__(self, state_list):
        # outcomes here will look like goal_0, goal_1, goal_2
        State.__init__(self, outcomes=['goal_{}'.format(i) for i in range(len(state_list))],
                            input_keys=['next_state'])
    def execute(self, userdata):
        return userdata.next_state

class MoveRoverToCoord(State):
    def __init__(self, coord):
        State.__init__(self, outcomes=['ok', 'err'], input_keys=['coord'])
        self.x = coord[0]
        self.y = coord[1]
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo('Waiting for move_base action server...')
        self.client.wait_for_server()
        rospy.loginfo('Found.')

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(goal)
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'ok'
        else:
            return 'err'

class SmWrapper():
    def __init__(self, state_list):
        self.top_level_state_machine = smach.StateMachine(outcomes=['done'])

        with self.top_level_state_machine:
            self.sequence_state_machine = smach.StateMachine(outcomes=['done', 'err'])
            smach.StateMachine.add('SEQUENCE', self.sequence_state_machine, {'done':'done','err':'ERROR'})
            smach.StateMachine.add('ERROR', Error(), transitions={'done':'done', 'seq':'SEQ'})

            with self.sequence_state_machine:
                # the transitions for 'PROCESS_USER_DATA' state will look like {goal_0:GOAL_0, goal_1:GOAL_1, etc.}
                transitions = {}
                for i in range(len(state_list)):
                    goal_name = 'goal_{}'.format(i)
                    transitions[goal_name] = goal_name.upper()
                smach.StateMachine.add('PROCESS_USER_DATA', ProcessUserData(state_list), transitions)


                for i in range(len(state_list)):
                    goal_transitions = {}
                    if i == (len(state_list)-1):
                        goal_transitions = {'ok':'done', 'err':'err'}

                    else:
                        goal_transitions = {'ok':'GOAL_{}'.format(i+1), 'err':'err'}


                    if state_list[i]['type'] == 'move':
                        smach.StateMachine.add('GOAL_{}'.format(i), MoveRoverToCoord(state_list[i]['coord']), goal_transitions)

                    # we need to add if statements for other states such as the one with type 'arm'


    def execute(self):
        return self.top_level_state_machine.execute()


def main():
    rospy.init_node('mission_planner')
    # sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok', input_keys=['coord'])
    # with sq:
    #     Sequence.add('move-to-coord', MoveRoverToCoord())
    state_list = [{'type': 'move', 'coord':(1.0, 0.0)}, {'type': 'move', 'coord':(1.5, 0.2)}, {'type': 'move', 'coord':(2.0, 0.3)} ]
    planner = SmWrapper(state_list)
   # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = planner.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
