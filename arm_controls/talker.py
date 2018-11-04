#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

class WidgetContainer(GridLayout):


    def sendValues(self, oldValues):
        # Send values to rover here. As of now this function only prints values to console
        # This function doesn't do anything unless the values change from the last run

        # Populate array of updated values
        newValues = [
                self.valueX.value,
                self.valueY.value,
                self.valueZ.value,
                self.valueF.value,
                self.valueG.value,
                self.valueW.value,
        ]

        # Check if any changes have been made to values since last fun. If any changes have been made then update values
        if (newValues != oldValues):    
            decPrecision = PanelApp.floatNum # Decimal precision of value report

            # print values to console
            print 'Values updated at ' + str(datetime.datetime.now().time())
            print '    X: ' + str(round(newValues[0], decPrecision))
            print '    Y: ' + str(round(newValues[1], decPrecision))
            print '    Z: ' + str(round(newValues[2], decPrecision))
            print 'Flick: ' + str(round(newValues[3], decPrecision))
            print ' Grab: ' + str(round(newValues[4], decPrecision))
            print 'Wrist: ' + str(round(newValues[5], decPrecision))
            oldValues = newValues # update array for next run

        PanelApp.valueArray = oldValues

     # Get the joystick events
     # Make sure to have xboxdrv installed
    def __init__(self, **kwargs):
        super(WidgetContainer, self).__init__(**kwargs)

        # get joystick events first
        Window.bind(on_joy_button_up=self.control_option)
        Window.bind(on_joy_axis=self.z_control)
        Window.bind(on_joy_axis=self.x_control)
        Window.bind(on_joy_axis=self.flick_control)


    def control_option(self, win, stickid, release_id):
        if release_id == 12:
            Window.unbind(on_joy_axis=self.x_control)
            Window.bind(on_joy_axis=self.y_control)
        elif release_id == 11:
            Window.unbind(on_joy_axis=self.y_control)
            Window.bind(on_joy_axis=self.x_control)

        if release_id == 1:
            Window.unbind(on_joy_axis=self.flick_control)
            Window.unbind(on_joy_axis=self.wrist_control)
            Window.bind(on_joy_axis=self.grab_control)

        elif release_id == 2:
            Window.unbind(on_joy_axis=self.flick_control)
            Window.unbind(on_joy_axis=self.grab_control)
            Window.bind(on_joy_axis=self.wrist_control)

        elif release_id == 3:
            Window.unbind(on_joy_axis=self.grab_control)
            Window.unbind(on_joy_axis=self.wrist_control)
            Window.bind(on_joy_axis=self.flick_control)

    def x_control(self, win, stickid, axisid, value):
        if value > 0 and axisid == 0:
            if self.valueX.value < 100:
                self.valueX.value += 1
        elif value < 0 and axisid == 0:
            if self.valueX.value > -100:
                self.valueX.value -= 1


    def y_control(self, win, stickid, axisid, value):
        if value < 0 and axisid == 1:
            if self.valueY.value < 100:
                self.valueY.value += 1
        elif value > 0 and axisid == 1:
            if self.valueY.value > -100:
                self.valueY.value -= 1
            
    def z_control(self, win, stickid, axisid, value):
        if value < 0 and axisid == 4:
            if self.valueZ.value < 100:
                self.valueZ.value += 1
        elif value > 0 and axisid == 4:
            if self.valueZ.value > -100:
                self.valueZ.value -= 1
             
    def flick_control(self, win, stickid, axisid, value):
        if axisid == 5:
            if self.valueF.value < 100:
                self.valueF.value += 1
        elif axisid == 2:
            if self.valueF.value > -100:
                self.valueF.value -= 1

    def grab_control(self, win, stickid, axisid, value):
        if axisid == 5:
            if self.valueG.value < 100:
                self.valueG.value += 1
        elif axisid == 2:
            if self.valueG.value > -100:
                self.valueG.value -= 1

    def wrist_control(self, win, stickid, axisid, value):
        if axisid == 5:
            if self.valueW.value < 360:
                self.valueW.value += 1
        elif axisid == 2:
            if self.valueW.value > -360:
                self.valueW.value -= 1
def talker():

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass