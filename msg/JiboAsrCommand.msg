# Message for telling the ASR node to start or stop streaming audio to Google
# for processing.

# Standard ROS header.
Header header

# Listen for "Hey, Jibo" first.
bool heyjibo

# Continuously listen and publish asr result.
bool continuous
bool incremental
bool alternatives

# Path to a rule file, e.g. "src/rules/test.rule"
# or a Jibo NLU rule script.
string rule

# Command. Use the constants defined below.
int8 command

# Commands. The ASR node can publish results (transcript plus confidence value)
# for three kinds of things:
# (1) the final result (i.e., the final result with the highest confidence),
# (2) interim results that are not final, with a stability measure indicating
#     how likely it is that the result will change, and
# (3) alternative results, which are other phrases that might be what you said,
#     but aren't as good a guess as the top alternative
# Thus, you can tell the ASR node to start or stop sending any of these.
int8 STOP = 0
int8 START = 1
