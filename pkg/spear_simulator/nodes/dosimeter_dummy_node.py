#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix

from math import sin

latitude = 51.4530
longitude = -112.7161


def gps_callback(fix):
    global latitude, longitude
    latitude = fix.latitude
    longitude = fix.longitude


def get_instantaneous_dose():
    """
    Generates a small, positive number that changes with the rover's position
    """
    return abs(sin(10000 * latitude)) + abs(sin(10000 * longitude))


def publish():
    pub_instantaneous = rospy.Publisher('dosimeter/instantaneous', Float64, queue_size=10)
    pub_accumulated = rospy.Publisher('dosimeter/accumulated', Float64, queue_size=10)
    freq_hz = 10
    rate = rospy.Rate(freq_hz)
    accumulated_dose = 0
    while not rospy.is_shutdown():
        instantaneous_dose = get_instantaneous_dose()
        # Integrate Sv/hr to Sv
        accumulated_dose += instantaneous_dose * (1 / freq_hz) / (60 ** 2)
        pub_accumulated.publish(accumulated_dose)
        pub_instantaneous.publish(instantaneous_dose)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('dosimeter_dummy_node', anonymous=True)
    rospy.Subscriber('gps/fix', NavSatFix, gps_callback)
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
