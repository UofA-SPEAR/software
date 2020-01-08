#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool

def talker():
    voltage = rospy.Publisher('web/battery/voltage', Float64, queue_size=10)
    motorArm = rospy.Publisher('web/motor/arm', Float64, queue_size=10)
    fuse1 = rospy.Publisher('web/fuse1', Bool, queue_size=10)
    power = rospy.Publisher('web/power/usage', Float64, queue_size=10)
    canwheel = rospy.Publisher('web/can/wheel', Bool, queue_size=10)


    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        rospy.loginfo("~~~~~~~~")

        batterymsg = round(10-rospy.get_time()%10,2)
        rospy.loginfo(batterymsg)
        voltage.publish(batterymsg)

        motormsg = round(10-rospy.get_time()%10,2)
        rospy.loginfo(motormsg)
        motorArm.publish(motormsg)

        fuse1msg = False
        rospy.loginfo(fuse1msg)
        fuse1.publish(fuse1msg)

        powermsg = round((15-rospy.get_time()%10),2)
        rospy.loginfo(powermsg)
        power.publish(powermsg)

        canwheelmsg = False
        rospy.loginfo(canwheelmsg)
        canwheel.publish(canwheelmsg)


        
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
