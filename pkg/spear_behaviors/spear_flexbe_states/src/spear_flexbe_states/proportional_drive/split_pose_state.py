from flexbe_core import EventState
from geometry_msgs.msg import PointStamped, QuaternionStamped


class SplitPoseState(EventState):
    '''
    Splits a goal pose into a goal position and a goal orientation.

    ># target_pose        PoseStamped          Input pose
    <# target_position    PointStamped         Output position
    <# target_orientation QuaternionStamped    Output orientation
    '''
    def __init__(self):
        super(SplitPoseState, self).__init__(outcomes=['ok'], input_keys=['target_pose'], output_keys=['target_position', 'target_orientation'])

    def on_enter(self, userdata):
        pose = userdata.target_pose  # type: PoseStamped
        position = PointStamped()
        position.header.frame_id = pose.header.frame_id
        position.point = pose.pose.position
        orientation = QuaternionStamped()
        orientation.header.frame_id = pose.header.frame_id
        orientation.quaternion = pose.pose.orientation
        userdata.target_position = position
        userdata.target_orientation = orientation

    def execute(self, ud):
        return 'ok'
