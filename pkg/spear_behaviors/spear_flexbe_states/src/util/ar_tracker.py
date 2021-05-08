from typing import Dict, Optional

from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import PoseStamped
from rospy import Subscriber


class ARTracker:
    def __init__(self, marker_topic, frame_id):  # type: (str, str) -> None
        self._marker_poses = {}  # type: Dict[int, PoseStamped]
        self._frame_id = frame_id
        self._subscriber = Subscriber(marker_topic, AlvarMarkers, self.markers_callback, queue_size=1)

    def markers_callback(self, message):  # type: (AlvarMarkers) -> None
        for marker in message.markers:  # type: AlvarMarker
            self._marker_poses[marker.id] = marker.pose
            # ar_track_alvar doesn't correctly add the pose frame id, so we need to add it here
            self._marker_poses[marker.id].header.frame_id = self._frame_id

    def get_pose(self, marker_id):  # type: (int) -> Optional[PoseStamped]
        if marker_id not in self._marker_poses:
            return None
        return self._marker_poses[marker_id]

    def has_seen(self, marker_id):  # type: (int) -> bool
        return marker_id in self._marker_poses

    def unregister(self):
        self._subscriber.unregister()
