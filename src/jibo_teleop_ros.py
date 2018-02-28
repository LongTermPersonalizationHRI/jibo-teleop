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
import rospy # ROS
from jibo_msgs.msg import JiboAction # ROS msgs to talk to Tega
from r1d1_msgs.msg import TegaState # ROS msgs to get info from Tega
from std_msgs.msg import Bool # for child_attention topic
from std_msgs.msg import Header # standard ROS msg header

class jibo_teleop_ros():
    # ROS node

    def __init__(self, ros_node, ros_label, flags):
        """ Initialize ROS """
        # we get a reference to the main ros node so we can do callbacks
        # to publish messages, and subscribe to stuff
        self.ros_node = ros_node
        # we're going to update the ros label with info about messages coming
        # in one topics we're subscribed to
        self.ros_label = ros_label
        # these are shared flags that the UI code will use to change the colors
        # of text or buttons based on what messages we're getting
        self.flags = flags
        # subscribe to other ros nodes
        #TODO could we put list of nodes to subscribe to in config file?
        # the child attention topic gives us a boolean indicating whether or
        # not the affdex camera is recognizing a person's face looking in
        # generally the right direction
        rospy.Subscriber('child_attention', Bool, self.on_child_attn_msg)
        rospy.Subscriber('tega_state', TegaState, self.on_tega_state_msg)

        # We will publish commands to the tablet and commands to the robot.
        # We might send audio to the audio entrainer on its way to the robot.
        # TODO it may be worthwhile to put the topic names in the config file.
        # self.tablet_pub = rospy.Publisher('opal_tablet_command', OpalCommand,
        #         queue_size = 10)
        self.jibo_pub = rospy.Publisher('jibo', JiboAction, queue_size = 10)


    # def send_opal_message(self, command):
    #     """ Publish opal command message """
    #     if self.tablet_pub is not None:
    #         print('sending opal command: %s' % command)
    #         msg = OpalCommand()
    #         # add header
    #         msg.header = Header()
    #         msg.header.stamp = rospy.Time.now()
    #         msg.command = command
    #         self.tablet_pub.publish(msg)
    #         rospy.loginfo(msg)

    def send_motion_message(self, motion):
        """ Publish JiboAction do motion message """
        if self.jibo_pub is not None:
            print('sending motion message: %s' % motion)
            msg = JiboAction()
            # add header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.do_motion = True
            msg.motion = motion
            self.jibo_pub.publish(msg)
            rospy.loginfo(msg)

    def send_lookat_message(self, lookat):
        """ Publish JiboAction lookat message """
        if self.jibo_pub is not None:
            print('sending lookat message: %s' % lookat)
            msg = JiboAction()
            # add header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.do_lookat = True
            msg.lookat = lookat
            self.jibo_pub.publish(msg)
            rospy.loginfo(msg)
    
    def send_sound_message(self, speech):
        """ Publish JiboAction playback audio message """
        if self.jibo_pub is not None:
            print('\nsending sound message: %s' % speech)
            msg = JiboAction()
            # add header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.do_sound_playback = True
            msg.audio_filename = speech
            self.jibo_pub.publish(msg)
            rospy.loginfo(msg)

    def send_speech_message(self, speech):
        """ Publish JiboAction playback TTS message """
        if self.jibo_pub is not None:
            print('\nsending speech message: %s' % speech)
            msg = JiboAction()
            # add header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.do_tts = True
            msg.tts_text = speech
            self.jibo_pub.publish(msg)
            rospy.loginfo(msg)

    def send_fidget_message(self, fidget):
        """ Publish TegaAction message setting the fidget set in use. """
        if self.tega_pub is not None:
            print('\nsending fidget message: %s' % fidget)
            msg = TegaAction()
            # add header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.fidgets = fidget
            self.tega_pub.publish(msg)
            rospy.loginfo(msg)

    def send_volume_message(self, volume):
        """ Publish TegaAction message setting the percent volume to use. """
        if self.tega_pub is not None:
            print('\nsending volume message: %s' % volume)
            msg = TegaAction()
            # add header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.set_volume = True
            msg.percent_volume = volume
            self.tega_pub.publish(msg)
            rospy.loginfo(msg)

    def on_child_attn_msg(self, data):
        # when we get child attention messages, set a label to say whether the
        # child is attending or not, and also set a flag
        self.flags.child_is_attending = data.data
        if data.data:
            self.ros_label.setText("Child is ATTENDING")
        else:
            self.ros_label.setText("Child is NOT ATTENDING")

    def on_tega_state_msg(self, data):
        # when we get tega state messages, set a flag indicating whether the
        # robot is in motion or playing sound or not
        self.flags.tega_is_playing_sound = data.is_playing_sound

        # Instead of giving us a boolean to indicate whether tega is in motion
        # or not, we get the name of the animation. Let's check whether it is
        # our "idle" animation (usually, the idle animation is either
        # MOTION_IDLESTILL or MOTION_BREATHING).
        self.flags.tega_is_doing_motion = data.doing_motion
