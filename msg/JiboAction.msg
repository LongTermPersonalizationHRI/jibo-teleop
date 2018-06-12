#
# Message for requesting actions of Jibo controller
#

Header header

bool do_motion
string motion  # use constants defined below

bool do_lookat
JiboVec3 lookat

bool do_tts
string tts_text

bool do_sound_playback
string audio_filename

bool do_led
JiboVec3 led_color

bool do_volume
float32 volume

bool do_anim_transition
int8 anim_transition # send ANIMTRANS_KEEP_LASTFRAME to have the last frame of an animation stay on Jibo's eye after playback finishes, 0 to reset 

#
# TTS Options
#
float32 tts_duration_stretch # Multiplier, so 2.0 = twice as slow, .5 = twice as fast
float32 tts_pitch # Valid range is 0.2 - 0.8

#
# Anim Transition Constants
#
int8 ANIMTRANS_RESET = 0
int8 ANIMTRANS_KEEP_LASTFRAME = 1

#
# Animation Constants
#

string EMOJI_SHARK = Emoji/Emoji_Shark_01_01.keys
string EMOJI_BEER = Emoji/Emoji_Beer_01_01.keys
string EMOJI_PARTY_PINK = Emoji/Emoji_PartyPink_01_01.keys
string EMOJI_PARTY_BLUE = Emoji/Emoji_PartyBlue_01_01.keys
string EMOJI_RAINCLOUD = Emoji/Emoji_RainCloud_01_01.keys

string RING_IN_ANIM = Misc/ring_in.keys
string HAPPY_GO_LUCKY_DANCE = Dances/Happy_Lucky_01_01.keys
string EYE_FIDGET = Misc/eye_fidget.keys
string LOOK_DOWN = Poses/Directional/Body_Look_Center_Down_01_01.keys
string DEFAULT = default.keys

# Animations for interactive storybook.
string QUESTION_ANIM_AND_EMOJI = Misc/Question_01_02.keys
string HAPPY_ANIM_AND_SOUND = Misc/Eye_to_Happy_01.keys
string CELEBRATE_ANIM = Dances/Celebrate_01.keys
string LAUGH_ANIM = Misc/Laughter_01_03.keys
string SAD_ANIM_SHAKE_HEAD = Misc/Sad_03.keys

# Animations for iSpy Game
string SILENT_NOD = silent/youre_welcome_02.keys
string SILENT_HAPPY_DANCE = silent/side_shaker_01_01.keys
string SILENT_YES = silent/nod_twice_01.keys
string SILENT_PUZZLED = silent/confused_01.keys
string SILENT_FRUSTRATED = silent/frustrated_02.keys
string SILENT_SAD = silent/sad_03.keys
string SILENT_INTERESTED = silent/interested_02.keys
string SILENT_EXCITED = silent/excited_03.keys



#
# Motion Constants
#
string RING_IN_SOUND = SFX_Global_CheckmarkButton.m4a
