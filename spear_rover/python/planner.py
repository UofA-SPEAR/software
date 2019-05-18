#!/usr/bin/python
from smach import State, StateMachine, Sequence

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from spear_msgs.msg import ArmAngles
from time import sleep


class MoveRoverForward(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'err'])
        self.publisher = rospy.Publisher('/rover_diff_drive_controller/cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):
        twist = Twist(
            linear=Vector3(10, 0, 0),
            angular=Vector3(0, 0, 0),
        )
        for _ in range(100):
            self.publisher.publish(twist)
            sleep(0.1)
        return 'ok'


class TurnRoverClockwise(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'err'])
        self.publisher = rospy.Publisher('/rover_diff_drive_controller/cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):
        twist = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, -3.1415926535897/2),
        )
        for _ in range(100):
            self.publisher.publish(twist)
            sleep(0.1)
        return 'ok'


def main():
    rospy.init_node('mission_planner')
    sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
    with sq:
        Sequence.add('forward', MoveRoverForward())
        Sequence.add('turn', TurnRoverClockwise(), transitions={'ok': 'forward'})

    sq.execute()


if __name__ == '__main__':
    main()
