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
from jibo_msgs.msg import JiboAction # ROS msgs
from jibo_teleop_ros import jibo_teleop_ros
from functools import partial

class jibo_animation_ui(QtGui.QWidget):

   # List of animations for Tega. Not all the SILENT animations are here.
    animations = [
            JiboAction.EMOJI_SHARK,
            JiboAction.EMOJI_BEER,
            JiboAction.EMOJI_PARTY_PINK,
            JiboAction.EMOJI_PARTY_BLUE,
            JiboAction.EMOJI_RAINCLOUD,
            JiboAction.HAPPY_GO_LUCKY_DANCE
            ]

    def __init__(self, ros_node):
        """ Make a button for each animation """
        super(jibo_animation_ui, self).__init__()
        # get reference to ros node so we can do callbacks to publish messages
        self.ros_node = ros_node

        # put buttons in a box
        anim_box = QtGui.QGroupBox(self)
        anim_layout = QtGui.QGridLayout(anim_box)
        anim_box.setTitle("Animations")

        # create animation buttons and add to layout
        col = 0
        row = 1
        for anim in self.animations:
            button = QtGui.QPushButton(anim.lower().replace("\"", ""), anim_box)
            button.clicked.connect(partial(self.ros_node.send_motion_message, anim))
            # if in the top left, make button green
            if (col < 3 and row < 7):
                button.setStyleSheet('QPushButton {color: green;}')
            # if in top right, make button red
            if (col > 2 and row < 3):
                button.setStyleSheet('QPushButton {color: red;}')
            anim_layout.addWidget(button, row, col)
            col += 1
            if(col >= 4): # ten animation buttons per row
                col = 0
                row += 1
