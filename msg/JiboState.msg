Header header

# This will list the current animation that Tega is in.
# See JiboAction.msg for possible values.
# The default idle animation is usually MOTION_IDLESTILL
# or MOTION_POSE_BREATHING, but it is not guaranteedd to
# be, since this is set in the robot code.
string in_motion
string audio
string tts_msg
float32 volume
#JiboVec3 led_color
#JiboVec3 lookat

# True when the robot is performing an animation that is not the 
# robot's IDLE animation       {}        
bool doing_motion               

# True when the sound player is playing back a sound on Tega 
bool is_playing_sound

# True when Jibo is listening to speech
bool is_listening