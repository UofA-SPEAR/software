from flexbe_core import EventState
from geometry_msgs.msg import Point, Quaternion, PoseStamped

from util.globals import gates
from util.math import normal_to_quaternion
from ros_numpy import numpify, msgify


class SetGoalToGateNormalOffsetState(EventState):
    '''
    Output a navigation goal to the given post as a userdata

    ># leg_number int Which leg of the course the gate is at (4 to 7)
    <# target_pose PoseStamped  A pose in front of and facing the gate

    -- offset float Distance from center of the gate to set the goal; positive for along normal.

    <= ok  Goal is correctly set
    <= not_seen Gate has not been seen
    '''
    def __init__(self, offset):  # type: (float) -> None
        super(SetGoalToGateNormalOffsetState, self).__init__(outcomes=['ok', 'err'], input_keys=['leg_number'], output_keys=['target_pose'])
        self._offset = offset
        self._error = False

    def on_enter(self, userdata):
        gate = gates[userdata.leg_number]
        if not gate.has_seen():
            self._error = True
            return
        point = numpify(gate.position.point)
        normal = numpify(gate.normal.vector)
        target_pose = PoseStamped()
        target_pose.header.frame_id = gate.position.header.frame_id
        target_pose.pose.position = msgify(Point, point + normal * self._offset)
        target_pose.pose.orientation = msgify(Quaternion, normal_to_quaternion(-normal))
        userdata.target_pose = target_pose

    def execute(self, ud):
        if self._error:
            return 'not_seen'
        else:
            return 'ok'
