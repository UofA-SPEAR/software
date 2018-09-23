import kivy

from kivy.app import App

from kivy.uix.gridlayout import GridLayout

from kivy.uix.label import Widget
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.properties import NumericProperty


class WidgetContainer(GridLayout):

    def __init__(self, **kwargs):
        super(WidgetContainer, self).__init__(**kwargs)

        self.cols = 4

        self.slide_x = Slider(min=-100, max=100, value=0)
        self.slide_y = Slider(min=-100, max=100, value=0, orientation='vertical')
        self.slide_z = Slider(min=-100, max=100, value=0, orientation='vertical')

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

        self.slide_x.bind(value=self.on_valueX)
        self.slide_y.bind(value=self.on_valueY)
        self.slide_z.bind(value=self.on_valueZ)

    def on_valueX(self, instance, inValue):
        self.ValueX.text = "%d"%inValue

    def on_valueY(self, instance, inValue):
        self.ValueY.text = "%d"%inValue
    
    def on_valueZ(self, instance, inValue):
        self.ValueZ.text = "%d"%inValue

class ArmInterface(App):

    def build(self):
        widgetcontainer = WidgetContainer()

        return widgetcontainer




if __name__ == '__main__':
    ArmInterface().run()
