import kivy

from kivy.app import App

from kivy.uix.gridlayout import GridLayout

from kivy.core.window import Window
from kivy.uix.label import Widget
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from kivy.uix.textinput import TextInput
from kivy.uix.slider import Slider
from kivy.properties import NumericProperty, ListProperty, ObjectProperty, StringProperty

# We will be using the knob class from kivy garden to add a control knob to our app.
# Use 'pip install kivy-garden' and then 'garden install knob' to install the knob class
# See here: https://github.com/kivy-garden/garden.knob
from kivy.garden.knob import Knob



class WidgetContainer(GridLayout):
    pass
# Define our app
class PanelApp(App):
    vSlideWidth = NumericProperty(60)
    hGridHeight = NumericProperty(Window.size[1]/6)
    labelHeight = NumericProperty(20)
    tinputHeight = NumericProperty(30)

    def build(self):
        self.title = 'SPEAR Arm Control Panel'
        self.icon = 'window_icon.ico'
        widgetcontainer = WidgetContainer()

        return widgetcontainer

    def displayInput(self, mySlideValue):
        return str(round(mySlideValue, 2))

    def checkInput(self, myText):
        try:
            newNum = float(myText)
        except ValueError:
            newNum = 0
        return newNum

class inputPopup(Popup):
    inputValue = StringProperty(None)

    pass


# Run app
if __name__ == '__main__':
    PanelApp().run()
