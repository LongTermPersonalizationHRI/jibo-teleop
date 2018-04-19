#!/usr/bin/env python

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

import sys # exit and argv
import argparse # command line args
import rospy # ROS
from PySide import QtGui, QtCore # basic GUI stuff
from jibo_msgs.msg import JiboAction # ROS msgs
from jibo_msgs.msg import JiboVec3 # ROS msgs

from jibo_teleop_ros import jibo_teleop_ros
from jibo_animation_ui import jibo_animation_ui
from jibo_lookat_ui import jibo_lookat_ui
from jibo_speech_ui import jibo_speech_ui
from jibo_volume_ui import jibo_volume_ui
from jibo_teleop_flags import jibo_teleop_flags
from AudioRecorder import AudioRecorder

class jibo_teleop(QtGui.QMainWindow):
    """ Tega teleoperation interface """
    # set up ROS node globally
    # TODO if running on network where DNS does not resolve local
    # hostnames, get the public IP address of this machine and
    # export to the environment variable $ROS_IP to set the public
    # address of this node, so the user doesn't have to remember
    # to do this before starting the node.
    ros_node = rospy.init_node('jibo_teleop', anonymous=True)

    def __init__(self, pid='test_user', experimenter='jibo'):
        """ Initialize teleop interface """
        # setup GUI teleop interface
        super(jibo_teleop, self).__init__()
        self.setGeometry(50, 50, 950, 1500)
        self.setWindowTitle("Jibo Teleop")

        # create layout
        self.central_widget = QtGui.QWidget(self)
        self.central_layout = QtGui.QGridLayout(self.central_widget)
        self.setCentralWidget(self.central_widget)

        # add label for ROS messages to update
        self.ros_label = QtGui.QLabel(self)
        self.ros_label.setText("---")
        self.central_layout.addWidget(self.ros_label, 3, 6, 1, 1,
            alignment=QtCore.Qt.AlignLeft)

        # we have a boolean to flag whether child is attending or not based
        # on data coming in on the /child_attention topic from ROS
        # TODO this is a project-specific flag - need to revise how this
        # is done so that project-specific stuff can be swapped out for
        # new projects
        self.flags = jibo_teleop_flags()

        # setup ROS node publisher and subscriber
        self.ros_teleop = jibo_teleop_ros(self.ros_node, self.ros_label,
               self.flags)

        # add animation buttons
        anim_ui = jibo_animation_ui(self.ros_teleop)
        self.central_layout.addWidget(anim_ui, 0, 0, 4, 10)

        # Add robot script playback buttons (mostly speech, but the scripts
        # can also list animations to play before or after an audio file).
        speech_ui = jibo_speech_ui(self.ros_teleop, self.flags, pid, experimenter)
        self.central_layout.addWidget(speech_ui, 1, 0, 4, 10)


        # add lookat buttons
        lookat_ui = jibo_lookat_ui(self.ros_teleop)
        self.central_layout.addWidget(lookat_ui, 4, 0, 2, 4)

        # Add fidget control buttons.
        # fidget_ui = tega_fidget_ui(self.ros_teleop)
        # self.central_layout.addWidget(fidget_ui, 4, 3, 1, 2)

        # Add volume controls.
        volume_ui = jibo_volume_ui(self.ros_teleop)
        self.central_layout.addWidget(volume_ui, 5, 0, 1, 3)



if __name__ == '__main__':

    print('hello')

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='''Send commands to a Jibo robot. Creates
            a Qt GUI with buttons for triggering speech, lookats, and
            animations on the robot, as well as commands to send to an Opal
            device. Support the ability to load custom scripts for robot behavior.
            ''')

    parser.add_argument('pid', type=str, nargs=1, help='the participant ID for a given session')
    parser.add_argument('experimenter', type=str, nargs=1, help='the participant ID for a given session')

    # Get arguments.
    args = parser.parse_args()
    print("ARGS WERE")
    print(args)

    # initialize top-level GUI manager
    app = QtGui.QApplication(sys.argv[0])

    # start teleop interface
    try:
        teleop_window = jibo_teleop(args.pid[0], args.experimenter[0])
        teleop_window.show()

    # if roscore isn't running or shuts down unexpectedly
    except rospy.ROSInterruptException:
        print ('ROS node shutdown')
        pass

    # enter main loop, then exit
    sys.exit(app.exec_())
