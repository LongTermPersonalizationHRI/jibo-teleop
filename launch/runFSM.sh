#!/bin/bash
echo -e "\033[1m\033[42mCTRL-C here to stop all\033[0m"
echo


#gnome-terminal --geometry 93x31+0+900 --title "Speech Recognition" -e "./tega/cognition/node_pyaudio_google_asr.py" &
#xterm -geometry 45x20+300+0 -T "Speech Recognition" -e bash -c "./tega/cognition/node_pyaudio_google_asr.py" &
#xterm -geometry 45x20+300+0 -T "Speech Recognition" -e bash -c "./tega/cognition/node_google_asr.py" &


sleep 0.5s
echo "IM HERE"
#python -m scripts.start_tap_game_controller '$1' '$2' '$3'
xterm -geometry 45x20+200+0 -T "Main FSM" -e bash -c "python -m scripts.start_tap_game_controller '$1' '$2' '$3'" &
xterm -geometry 45x20+200+0 -T "USB Cam" -e bash -c "roslaunch usb_cam usb_cam-test.launch"
#xterm -geometry 45x20+300+350 -T "Affect Recognition" -e bash -c $CATKIN_DIR/devel/lib/affect_processing/affect_processing &
#xterm -geometry 45x20+600+350 -T "Affect Result" -e bash -c 'python tega/cognition/affect_interaction.py'
#xterm --title "Face Recognition" -e "python tega/cognition/face_recognition.py train=false"
#xterm --title "Speech Recognition" -e "python tega/cognition/node_google_asr.py"


