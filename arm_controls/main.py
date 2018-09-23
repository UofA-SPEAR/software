import kivy

from kivy.app import App

from kivy.uix.gridlayout import GridLayout

from kivy.uix.label import Widget
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.properties import NumericProperty

# We will be using the knob class from kivy garden to add a control knob to our app.
# Use 'pip install kivy-garden' and then 'garden install knob' to install the knob class
# See here: https://github.com/kivy-garden/garden.knob
# from kivy.garden.knob import Knob


class WidgetContainer(GridLayout):

    def __init__(self, **kwargs):
        super(WidgetContainer, self).__init__(**kwargs)

        # Set the column width for GridLayout
        self.cols = 4

        # Define sliders and values for each arm function
        self.slide_x = Slider(min=-100,max=100,value=0)
        self.slide_y = Slider(min=-100,max=100,value=0,orientation='vertical')
        self.slide_z = Slider(min=-100,max=100,value=0,orientation='vertical')

        # Add sliders and labels to grid
        self.add_widget(Label(text='X Position'))
        self.add_widget(self.slide_x)
        self.add_widget(Label(text="X Value"))
        self.ValueX = Label(text='0')
        self.add_widget(self.ValueX)

        self.add_widget(Label(text='Y Position'))
        self.add_widget(self.slide_y)
        self.add_widget(Label(text="Y Value"))
        self.ValueY = Label(text='0')
        self.add_widget(self.ValueY)

        self.add_widget(Label(text='Z Position'))
        self.add_widget(self.slide_z)
        self.add_widget(Label(text="Z Value"))
        self.ValueZ = Label(text='0')
        self.add_widget(self.ValueZ)

        # bind function to refresh label with value of slider
        self.slide_x.bind(value=self.on_valueX)
        self.slide_y.bind(value=self.on_valueY)
        self.slide_z.bind(value=self.on_valueZ)

        # Assign value to label for each slider
        # Maybe there is a better way to do this???
    def on_valueX(self, instance, inValue):
        self.ValueX.text = "%d"%inValue

    def on_valueY(self, instance, inValue):
        self.ValueY.text = "%d"%inValue
    
    def on_valueZ(self, instance, inValue):
        self.ValueZ.text = "%d"%inValue


# Define our app
class PanelApp(App):

    def build(self):
        self.title = 'SPEAR Arm Control Panel'
        self.icon = 'window_icon.ico'
        widgetcontainer = WidgetContainer()

        return widgetcontainer



# Run app
if __name__ == '__main__':
    PanelApp().run()
