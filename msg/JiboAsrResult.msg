# Message for sending automatic speech recognition results.

# Standard ROS header.
Header header

# The top ASR result.
string  transcription

# The reported confidence level that this result is correct.
float64 confidence

# Score for how well the text input parsed against the rule.
float64 heuristic_score

# Label associated to NLParse result according to given parse rule.
string slotAction
