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
from jibo_msgs.msg import JiboVec3 # ROS msgs to talk to Tega
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

        # We will publish commands to the tablet and commands to the robot.
        # We might send audio to the audio entrainer on its way to the robot.
        # TODO it may be worthwhile to put the topic names in the config file.
        # self.tablet_pub = rospy.Publisher('opal_tablet_command', OpalCommand,
        #         queue_size = 10)
        self.jibo_pub = rospy.Publisher('jibo', JiboAction, queue_size = 10)


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

    def send_tts_message(self, speech):
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


    def send_volume_message(self, volume):
        """ Publish JiboAction message setting the percent volume to use. """
        if self.jibo_pub is not None:
            print('\nsending volume message: %s' % volume)
            msg = JiboAction()
            # add header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.do_volume = True
            msg.volume = volume
            self.jibo_pub.publish(msg)
            rospy.loginfo(msg)

    def send_anim_transition_message(self, anim_transition):
        """ Publish JiboAction message that switches between animation playback modes. """
        if self.jibo_pub is not None:
            print('\nsending anim transition message: %s' % anim_transition)
            msg = JiboAction()
            # add header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.do_anim_transition = True
            msg.anim_transition = anim_transition
            self.jibo_pub.publish(msg)
            rospy.loginfo(msg)

    def send_led_message(self, red_val, green_val, blue_val):
        """ Publish JiboAction message that switches between animation playback modes. """
        if self.jibo_pub is not None:
            print('\nsending rgb_val message: %s' % str(red_val) + ',' + str(green_val) + ',' + str(blue_val))
            msg = JiboAction()
            # add header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.do_led = True
            msg.led_color = JiboVec3(red_val, green_val, blue_val)
            self.jibo_pub.publish(msg)
            rospy.loginfo(msg)

    def on_child_attn_msg(self, data):
        # when we get child attention messages, set a label to say whether the
        # child is attending or not, and also set a flag
        self.flags.child_is_attending = data.data
        if data.data:
            self.ros_label.setText("Child is ATTENDING")
        else:
            self.ros_label.setText("Child is NOT ATTENDING")

    def on_jibo_state_msg(self, data):
        # when we get Jibo state message, set a flag indicating whether the
        # robot is in motion or playing sound or not
        self.flags.jibo_is_playing_sound = data.is_playing_sound

        # Instead of giving us a boolean to indicate whether Jibo is in motion
        # or not, we get the name of the animation.
        self.flags.jibo_is_doing_motion = data.doing_motion
