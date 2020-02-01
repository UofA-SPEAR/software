#!/usr/bin/env python
from actionlib import GoalStatus
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult


class NavigateState(EventState):
    '''
    Navigate to waypoints.

    -- linear float    Linear speed in m/s (positive is forward)
    -- angular float   Angular speed in rad/s (positive is left)
    -- duration float  Duration of the move

    ># target_pose Dict[str,float]   Pose to navigate to, a dictionary with the keys 'x', 'y', and 'frame_id'

    <= ok              Navigation complete with no errors
    <= err             Navigation errors were encountered

    '''
    def __init__(self):
        super(NavigateState, self).__init__(outcomes=['ok', 'err'],
                                            input_keys=['target_pose'])
        self._topic = 'move_base'
        self._client = ProxyActionClient({self._topic: MoveBaseAction})
        self._error = False

    def execute(self, ud):
        if self._error:
            return 'err'

        if self._client.has_result(self._topic):
            status = self._client.get_state(self._topic)  # type: GoalStatus
            return 'ok' if status == GoalStatus.SUCCEEDED else 'err'

    def on_enter(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = userdata.target_pose['frame_id']
        goal.target_pose.pose.position.x = userdata.target_pose['x']
        goal.target_pose.pose.position.y = userdata.target_pose['y']
        goal.target_pose.pose.orientation.w = 1.0

        self._error = False
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as err:
            Logger.logwarn('Could not send nav goal to move_base:\n{}'.format(err))
            self._error = True

    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
