from urdf_parser_py.urdf import URDF

__urdf = None


def get_urdf():  # type: () -> URDF
    global __urdf
    if __urdf is None:
        __urdf = URDF.from_parameter_server()
    return __urdf
