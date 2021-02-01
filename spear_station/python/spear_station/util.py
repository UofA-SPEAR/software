from tf2_ros import Buffer, TransformListener
from urdf_parser_py.urdf import URDF

__urdf = None


def get_urdf():  # type: () -> URDF
    global __urdf
    if __urdf is None:
        __urdf = URDF.from_parameter_server()
    return __urdf


__tf_buffer = None
__tf_listener = None


def get_tf_buffer():
    global __tf_buffer, __tf_listener
    if __tf_buffer is None:
        __tf_buffer = Buffer()
        __tf_listener = TransformListener(__tf_buffer)
    return __tf_buffer
