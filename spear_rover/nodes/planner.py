#!/usr/bin/python
import actionlib
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach import State, StateMachine, Sequence
from spear_msgs.msg import ArmAngles
from std_msgs.msg import String, Header
from time import sleep


class MoveRoverToCoord(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'err'], input_keys=['coord'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo('Waiting for move_base action server...')
        self.client.wait_for_server()
        rospy.loginfo('Found.')

        x, y = userdata.coord
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(goal)
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_result() == actionlib.GoalStatus.SUCCEEDED:
            return 'ok'
        else:
            return 'err'


def main():
    rospy.init_node('mission_planner')
    sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok', input_keys=['coord'])
    with sq:
        Sequence.add('move-to-coord', MoveRoverToCoord())

    sq.execute({'coord': (1.0, 1.0)})


if __name__ == '__main__':
    main()
