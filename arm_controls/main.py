import kivy

from kivy.app import App
from kivy.uix.label import Widget
from kivy.uix.slider import Slider

s = Slider(min=-100, max=100, value=25)

class ArmInterface(App):

    def build(self):
        return s




if __name__ == '__main__':
    ArmInterface().run()
