# Jacqueline Kory Westlund
# May 2016
#
# The MIT License (MIT)
#
# Copyright (c) 2016 Personal Robots Group
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from PySide import QtGui # basic GUI stuff
from tega_teleop_ros import tega_teleop_ros
from sar_opal_msgs.msg import OpalCommand # ROS msgs

class opal_tablet_ui(QtGui.QWidget):
    def __init__(self, ros_node):
        """ Make buttons to send commands to opal tablet """
        super(opal_tablet_ui, self).__init__()
        # get reference to ros node so we can do callbacks to publish messages
        self.ros_node = ros_node

        # put buttons in a box
        opal_box = QtGui.QGroupBox(self)
        opal_layout = QtGui.QGridLayout(opal_box)
        opal_box.setTitle("Tablet Commands")

        # create opal command buttons and add to layout
        # next page
        self.nbutton = QtGui.QPushButton("next page", opal_box)
        self.nbutton.clicked.connect(lambda: self.ros_node.send_opal_message(
            OpalCommand.NEXT_PAGE))
        opal_layout.addWidget(self.nbutton, 0, 0)
        # previous page
        self.pbutton = QtGui.QPushButton("previous page", opal_box)
        self.pbutton.clicked.connect(lambda: self.ros_node.send_opal_message(
            OpalCommand.PREV_PAGE))
        opal_layout.addWidget(self.pbutton, 0, 1)
        # enable touch
        self.ebutton = QtGui.QPushButton("enable touch", opal_box)
        self.ebutton.clicked.connect(lambda: self.ros_node.send_opal_message(
            OpalCommand.ENABLE_TOUCH))
        opal_layout.addWidget(self.ebutton, 1, 0)
        # disable touch
        self.dbutton = QtGui.QPushButton("disable touch", opal_box)
        self.dbutton.clicked.connect(lambda: self.ros_node.send_opal_message(
            OpalCommand.DISABLE_TOUCH))
        opal_layout.addWidget(self.dbutton, 1, 1)
        # fade screen
        self.fbutton = QtGui.QPushButton("fade screen", opal_box)
        self.fbutton.clicked.connect(lambda: self.ros_node.send_opal_message(
            OpalCommand.FADE_SCREEN))
        opal_layout.addWidget(self.fbutton, 2, 0)
        # unfade screen
        self.sbutton = QtGui.QPushButton("unfade screen", opal_box)
        self.sbutton.clicked.connect(lambda: self.ros_node.send_opal_message(
            OpalCommand.UNFADE_SCREEN))
        opal_layout.addWidget(self.sbutton, 2, 1)
        # request keyframe
        self.kbutton = QtGui.QPushButton("request keyframe", opal_box)
        self.kbutton.clicked.connect(lambda: self.ros_node.send_opal_message(
            OpalCommand.REQUEST_KEYFRAME))
        opal_layout.addWidget(self.kbutton, 3, 1)
        # label for listing info
        self.label = QtGui.QLabel(opal_box)
        self.label.setText("")
        opal_layout.addWidget(self.label, 3, 2)


