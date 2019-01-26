import kivy
import datetime
from ros_station import SpinROS, ros_init

from kivy.app import App

from kivy.uix.gridlayout import GridLayout
from kivy.core.window import Window
from kivy.uix.label import Widget, Label
from kivy.uix.popup import Popup
from kivy.uix.textinput import TextInput
from kivy.uix.slider import Slider
from kivy.properties import (
    NumericProperty,
    ListProperty,
    ObjectProperty,
    StringProperty,
    ReferenceListProperty)
from kivy.clock import Clock
from threading import Thread


# We will be using the knob class from kivy garden to add a control knob to our app.
# Use 'pip install kivy-garden' and then 'garden install knob' to install
# the knob class
# See here: https://github.com/kivy-garden/garden.knob
from kivy.garden.knob import Knob


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


# Define our app
class PanelApp(App):
    # Set our variables for various operations
    vSlideWidth = NumericProperty(60) # width of vertical slider
    hGridHeight = NumericProperty(Window.size[1]/4) # height of bottom row
    labelHeight = NumericProperty(20) # height of all labels
    tinputHeight = NumericProperty(30) # height of all text boxes
    valueArray = [0,0,0,0,0,0] # set initial values for sendValues check
    floatNum = 2 # Set decimal precision for all calculations and reported values

    def build(self):
        self.title = 'SPEAR Arm Control Panel'
        self.icon = 'window_icon.ico'
        widgetcontainer = WidgetContainer()
        # Update changed values to be sent to the rover. Update 5 times per second
        Clock.schedule_interval(lambda dt: widgetcontainer.sendValues(self.valueArray), 1.0 / 5.0)
        return widgetcontainer

    def displayInput(self, mySlideValue):
        return str(round(mySlideValue, 2))

    def checkInput(self, myText):
        # Convert input from string to floating point
        try:
            newNum = round(float(myText),self.floatNum) # round number to desired precision
        except ValueError: # Report error instead of crashing if input was not a number
            errorPop = Popup(title='Error',
                    content=Label(text='Only numbers are allowed'),
                    size_hint=(None,None), size=(200,100))
            errorPop.open()
            newNum = 0
        return newNum


ros_thread = SpinROS()


# Run app
if __name__ == '__main__':
    ros_init()
    
    ros_thread.daemon = True
    ros_thread.start()

    PanelApp().run()
    