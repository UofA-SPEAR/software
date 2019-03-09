#! /usr/bin/env python

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.core.window import Window
from kivy.clock import Clock
from kivy.graphics import Color, Rectangle

from send_ros import SpinROS, ros_init, joyData, publish, Driver

# Key mappings for the drive system
keys = {
    "left_up": "i",
    "left_down": "k",
    "right_up": "o",
    "right_down": "l",
    "set_sync": "u",
    "set_mirror": "j",
    "set_split": "p",
    "pause": "a",
    "kill": "q",
}


# The UI widget for a single stick
# this would show either the left or right power
class DriveStick(Widget):
    def __init__(self, power=0, **kwargs):
        super(DriveStick, self).__init__(**kwargs)
        self.power = power

    # Changes the displayed power level
    def update_power(self):
        self.ids["nub"].pos = (self.pos[0], self.pos[1] + (self.height / 2.0) +
                               (self.power / 100.0 * self.height / 2.0) -
                               (self.ids["nub"].height / 2.0))


# The UI for the drive screen
# This displays the drive power level in both text and a visual
# representation for convinounce
class DriveScreen(Widget):
    SPLIT = 0
    SYNC = 1
    MIRROR = 2

    sensitivity = 50

    # Constructor
    # sets inital values for variables
    # creates listeners for keyboard
    # also sets schedulded function calls
    def __init__(self, **kwargs):
        super(DriveScreen, self).__init__(**kwargs)

        # setup variable
        self.mode = DriveScreen.SPLIT
        self.p_left = 0
        self.p_right = 0
        self.paused = True
        self.pressed = {
            "left_up": False,
            "left_down": False,
            "right_up": False,
            "right_down": False
        }
        self.frame = 0

        # setup ros node
        self.driver = Driver()

        # keyboard listener setup
        # self.controlmode = "joy"
        # if controlmode == "keyboard":
        self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
        self._keyboard.bind(on_key_down=self._on_keyboard_down)
        self._keyboard.bind(on_key_up=self._on_keyboard_up)
        # elif controlmode == "joy":
        # Clock.schedule_interval(self._update_joy, 0.01666666667)

        # scheduling function calls every n seconds
        Clock.schedule_interval(self._update_presses, 0.01666666667)

    # Sets the power to display, and transmit across ROS
    def power(self, left, right):
        self.p_left = left
        self.p_right = right

    # Sends the drive command through ros using our published
    def _send_drive(self):
        self.driver.send_cmd(int(self.p_left), int(self.p_right))

    # checks to see if you should send the drive command
    # if so, sends the command
    def _should_send_drive(self):
        self.frame += 1
        if self.frame % 6 == 0:
            self._send_drive()

    def _update_joy(self):
        if joyData.l_stick_y > 0:
            self.pressed["left_down"] = False
            self.pressed["left_up"] = True
        elif joyData.l_stick_y < 0:
            self.pressed["left_up"] = False
            self.pressed["left_down"] = True
        else:
            self.pressed["left_up"] = False
            self.pressed["left_down"] = False
        if joyData.r_stick_y > 0:
            self.pressed["right_down"] = False
            self.pressed["right_up"] = True
        elif joyData.r_stick_y < 0:
            self.pressed["right_up"] = False
            self.pressed["right_down"] = True
        else:
            self.pressed["right_up"] = False
            self.pressed["right_down"] = False
        if joyData.dpad[2] != 0:
            self.mode = DriveScreen.SYNC
        if joyData.dpad[0] != 0:
            self.mode = DriveScreen.MIRROR
        if joyData.dpad[3] != 0:
            self.mode = DriveScreen.SPLIT

    # updates left and right based on the press map
    # also calles the mirroring function
    def _update_presses(self, dt):
        if not self.paused:
            self._update_joy()
            if self.pressed["left_up"] != self.pressed["left_down"]:
                self.p_left += dt * DriveScreen.sensitivity * (
                    1 if self.pressed["left_up"] else -1)
            if self.pressed["right_up"] != self.pressed["right_down"]:
                self.p_right += dt * DriveScreen.sensitivity * (
                    1 if self.pressed["right_up"] else -1)
            self._do_mirroring()
            self._should_send_drive()
        if self.driver.is_ros_down():
            # it will only reach here if ros has shutdown
            App.get_running_app().Stop()  # this closes the app

    # release keyboard listener bindings
    def _keyboard_closed(self):
        self._keyboard.unbind(on_key_down=self._on_keyboard_down)
        self._keyboard.unbind(on_key_up=self._on_keyboard_up)
        self._keyboard = None

    # called when a key is released
    # used to update press map
    def _on_keyboard_up(self, keyboard, keycode):
        if keycode[1] == keys["left_up"]:
            self.pressed["left_up"] = False
        elif keycode[1] == keys["left_down"]:
            self.pressed["left_down"] = False
        elif keycode[1] == keys["right_up"]:
            self.pressed["right_up"] = False
        elif keycode[1] == keys["right_down"]:
            self.pressed["right_down"] = False
        else:
            return False

    # called when a key is pressed
    # used to update press map
    # also used for press commands
    def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
        if keycode[1] == keys["left_up"]:
            self.pressed["left_up"] = True
        elif keycode[1] == keys["left_down"]:
            self.pressed["left_down"] = True
        elif keycode[1] == keys["right_up"]:
            self.pressed["right_up"] = True
        elif keycode[1] == keys["right_down"]:
            self.pressed["right_down"] = True
        elif keycode[1] == keys["set_sync"]:
            self.mode = DriveScreen.SYNC
        elif keycode[1] == keys["set_mirror"]:
            self.mode = DriveScreen.MIRROR
        elif keycode[1] == keys["set_split"]:
            self.mode = DriveScreen.SPLIT
        elif keycode[1] == keys["pause"]:
            self.paused = not self.paused
            with self.ids["background"].canvas.before:
                if self.paused:
                    Color(0.7, 0.7, 0.7)
                else:
                    Color(0.96, 0.26, 0.21)
                Rectangle(
                    pos=self.ids["background"].pos,
                    size=self.ids["background"].size)
        elif keycode[1] == keys["kill"]:
            self.p_left = self.p_right = 0
        else:
            return False
        return True

    # manupulates the right power based on the left depending on the mode
    # also updates displayed power
    def _do_mirroring(self):
        # TODO: change so if left is not moving,
        # but right is, do the approprate actions
        self.p_left = max(-100, min(self.p_left, 100))
        self.p_right = max(-100, min(self.p_right, 100))
        if self.mode == DriveScreen.SPLIT:
            pass
        elif self.mode == DriveScreen.MIRROR:
            self.p_right = -self.p_left
        elif self.mode == DriveScreen.SYNC:
            self.p_right = self.p_left

        self._set_power(int(self.p_left), "left")
        self._set_power(int(self.p_right), "right")

    # Visually displays the power level of a given side
    def _set_power(self, power, side):
        self.ids[side[0] + "_stick"].power = power
        self.ids[side[0] + "_stick"].update_power()
        self.ids[side[0] + "_label"].text = (
            "[color=222222]{}: {}[/color]".format(side.capitalize(), power))


# Kivy application
# This is the main GUI for driving
class DriveApp(App):
    def build(self):
        return DriveScreen()


if __name__ == "__main__":
    ros_init()
    DriveApp().run()
