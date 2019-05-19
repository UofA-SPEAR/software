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
        self.client = actionlib.SimpleActionClient('move_base_simple/goal', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo('Waiting for move_base action server...')
        self.client.wait_for_server()

        header = Header(
            frame_id='base_link',
            stamp=rospy.Time.now(),
        )
        x, y = userdata.coord
        pos = Point(x, y, 0)
        orient = Quaternion(0, 0, 0, 1)
        goal = MoveBaseGoal(target_pose=PoseStamped(
            header=header,
            pose=Pose(position=pos, orientation=orient),
        ))
        rospy.loginfo(goal)
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_result() == actionlib.GoalStatus.SUCCEEDED:
            return 'ok'
        else:
            return 'err'


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
            angular=Vector3(0, 0, -3.1415926535897 / 2),
        )
        for _ in range(100):
            self.publisher.publish(twist)
            sleep(0.1)
        return 'ok'


def main():
    rospy.init_node('mission_planner')
    sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok', input_keys=['coord'])
    with sq:
        Sequence.add('forward', MoveRoverForward())
        Sequence.add('move-to-coord', MoveRoverToCoord())
        Sequence.add('turn', TurnRoverClockwise(), transitions={'ok': 'forward'})

    sq.execute({'coord': (1, 1)})


if __name__ == '__main__':
    main()
