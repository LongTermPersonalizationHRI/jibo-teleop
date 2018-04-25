#!/bin/bash
killall rosmaster
killall roslaunch
sleep 0.5s

WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')
xdotool windowfocus $WID
xdotool key ctrl+shift+t
wmctrl -i -a $WID

sleep 1; xdotool type --delay 1 --clearmodifiers "roscore"; xdotool key Return;


WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')
xdotool windowfocus $WID
xdotool key ctrl+shift+t
wmctrl -i -a $WID

sleep 2; xdotool type --delay 1 --clearmodifiers "roslaunch rosbridge_server rosbridge_websocket.launch"; xdotool key Return;
