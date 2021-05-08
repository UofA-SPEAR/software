from flexbe_core import EventState
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from util.globals import posts
from util.math import normal_to_quaternion
from ros_numpy import numpify, msgify


class SetGoalToPostState(EventState):
    '''
    Output a navigation goal to the given post as a userdata

    ># leg_number int Which leg of the course the post is at (1 to 3)
    <# target_pose PoseStamped  A pose in front of and facing the post

    <= ok  Goal is correctly set
    <= not_seen Post has not been seen
    '''
    def __init__(self):  # type: (int) -> None
        super(SetGoalToPostState, self).__init__(outcomes=['ok', 'err'], input_keys=['leg_number'], output_keys=['target_pose'])
        self._error = False

    def on_enter(self, userdata):
        post = posts[userdata.leg_number]
        if not post.has_seen():
            self._error = True
        target_pose = PoseStamped()
        point = numpify(post.position.point)
        normal = numpify(post.normal.vector)
        target_pose.header.frame_id = post.pose.header.frame_id
        target_pose.pose.position = msgify(Point, point + normal * 2)
        target_pose.pose.orientation = msgify(Quaternion, normal_to_quaternion(-normal))
        userdata.target_pose = target_pose

    def execute(self, ud):
        if self._error:
            return 'not_seen'
        else:
            return 'ok'
