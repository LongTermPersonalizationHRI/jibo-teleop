# jibo-teleop

This repository contains several tools for interfacing with the Jibo robot over ROS, including:

- ROS message definitions for communicating with the Jibo robot
- A Python ROS node and Qt GUI with buttons for triggering speech, lookats, and animations on the robot
- Scripts for teleoperating common interactions (e.g. the GFTA assessment and recording speech samples) via the teleop GUI 

## Installation

### Create virtualenv

If you haven't done this yet, set up and source a python3 virtual env with:
`virtualenv -p $(which python3) ~/python-virtualenvs/jibo-gfta-py3 --system-site-packages`

Then, activate it with
`source ~/python-virtualenvs/jibo-gfta-py3/bin/activate`


### Install System Dependencies
- `sudo apt-get install python3-pyside`
- `sudo apt-get install portaudio19-dev`
- `sudo apt-get install python3-dev`

### Install Python Dependencies
`pip install -r requirements.txt`

## Configure and Run

0. Activate the tap game virtualenv
```bashrc
source ~/python-virtualenvs/jibo-gfta-py3/bin/activate
```
1. Start `roscore` and `rosbridge` *# do not switch to a different terminal window until done loading*
```bashrc
$ ./launch_scripts/startROS.sh 
```

2. Start the Teleop
```bashrc
./src/jibo-teleop.py
```

2. Start the GFTA collection
```bashrc
./launch_scripts/start_gfta_collection.sh p00 sam no-record
```

3. Connect Jibo to ROS.
  - Note your `ROSMASTER_URI` from the host computer (where you started `roscore`. E.g., http://192.168.1.100:11311)
  - Run Jibo-Rosbridge-Receiver skill on the robot. (**input `ROS_IP`, e.g., 192.168.1.XXX**). 
  - Jibo should say "Connection Confirmed" when he connects to ROSBridge


### Config

The program will use values from the config file to load scripts and send audio:

    - script: the interaction script to use.
    - static_script: a list of "always there" speech buttons.

More detail about all these options is provided below.

Essentially, the program will try to read in the interaction script file listed
in the "jibo\_teleop\_config.txt" file. There is an example interaction script
file located in src/.


### More on scripts

#### Basic scripts

#### Animations in scripts

#### Static scripts

There is the option of including a set of "static script" buttons to trigger
speech. This is useful if, for example, the robot could always have the option
of saying "Mmhm" and "Awesome!" regardless of the rest of the script, and if
you don't want to include these phrases as options on every single line of your
main script file. Following the example\_static\_script.txt file, make a list
with one column of audio filenames and one tab-delimited column of button
labels.

You probably shouldn't list more than 3-5 of these "always there" speech
buttons, since otherwise the interface may start to look clunky with too many
buttons.

## ROS messages


### JIBO messages

The program publishes "/jibo\_msgs/JiboAction" to the ROS topic "/jibo".

The program subscribes to
"/[r1d1\_msgs](https://github.com/mitmedialab/r1d1_msgs
"/r1d1_msgs")/TegaState" on the ROS topic "/tega\_state".

See [/r1d1\_msgs](https://github.com/mitmedialab/r1d1_msgs "/r1d1_msgs") for
more info.

### Affdex attention messages

The program subscribes to Boolean (True/False) messages on the ROS topic
"/child\_attention". These messages indicate whether a child is attending to
the robot/tablet setup or not.


## Version Notes

This program was developed and tested with:

- Python 3.5.2
- ROS Kinetic
- Ubuntu 16.04 LTS (64-bit)

## Bugs and issues

Please report all bugs and issues on the [tega\_teleop github issues
page](https://github.com/mitmedialab/tega_teleop/issues).

## TODO

