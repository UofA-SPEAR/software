from util.ar_tracker import ARTracker
from util.post import Post, Gate

ar_tracker = ARTracker('/ar_pose_marker', 'odom')

# See https://7aec5dcb-a-3f6a8980-s-sites.googlegroups.com/a/marssociety.org/urc/home/q-a/ARgates.png

posts = {
    1: Post(0, ar_tracker),
    2: Post(1, ar_tracker),
    3: Post(2, ar_tracker),
}

gates = {
    4: Gate(Post(3, ar_tracker), Post(4, ar_tracker)),
    5: Gate(Post(5, ar_tracker), Post(6, ar_tracker)),
    6: Gate(Post(7, ar_tracker), Post(8, ar_tracker)),
    7: Gate(Post(9, ar_tracker), Post(10, ar_tracker)),
}
