#!/usr/bin/env python2

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QTabWidget


class ArmGuiPlugin(Plugin):
    def __init__(self, context):
        super(ArmGuiPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ArmGuiPlugin')

        # Create QWidget
        self._widget = QTabWidget()
        # Get path to UI file which should be in the "resource" folder of this
        # package
        ui_file = os.path.join(rospkg.RosPack().get_path('spear_station'),
                               'resource', 'ArmGuiPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ArmGuiPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO: unregister all publishers, timers, etc. here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO: save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO: restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
    #     # Comment in to signal that a plugin has a way to configure.
    #     # This will enable a setting button (gear icon) in each dock widget
    #     # title bar.
    #     # Usually used to open a modal configuration dialog.
