# jibo-teleop

- ROS message definitions for communicating with the Jibo robot
- A python rosnode for teleoperating the Jibo robot 
- Scripts for common interactions (e.g. the GFTA assessment and recording speech samples)

Creates a Qt GUI with buttons for triggering speech, lookats, and animations on the robot

## Installation

If you haven't done this yet, set up a python3 virtual env with:
`virtualenv -p $(which python3) ~/python-virtualenvs/jibo-gfta-py3 --system-site-packages`

YOU ONLY NEED TO DO THIS ONCE!

All subsequent times, you just need to activate it with
`source ~/python-virtualenvs/jibo-gfta-py3/bin/activate`

### Install System Dependencies
`sudo apt-get install python3-pyside`
`sudo apt-get install portaudio19-dev`
`sudo apt-get install python3-dev`

### Install Python Dependencies
`pip install -r requirements.txt`

## Configure and Run

`python tega_teleop.py [-h]

optional arguments:

    - `-h`, `--help`: show this help message and exit

On startup, this python node will try to connect to roscore. If roscore is not
running, the program will exit.

If this node is running on a network where DNS does not resolve local
hostnames, you will need to export the environment variables `$ROS_IP` and
`$ROS_HOSTNAME` to be the public IP address of this node. For example, if the
machine this node is running on has the IP address "192.168.1.20", you would
run the commands `export ROS_IP=192.168.1.20` and `export
ROS_HOSTNAME=192.168.1.20` in your shell prior to starting this node. If this
IP is static, you may want to put these commands in your bashrc file (or other
shell rc file)  so you don't have to remember to run them every time.

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

### Relational robot messages

The node publishes
"/[rr_msgs](https://github.com/mitmedialab/rr_msgs)/EntrainAudio" messages on
the ROS topic "/rr/entrain_audio".

The node also publishes
"/[rr_msgs](https://github.com/mitmedialab/rr_msgs)/InteractionState" messages
on the ROS topic "/rr/state".


## Version Notes

This program was developed and tested with:

- Python 3.5.2
- ROS Kinetic
- Ubuntu 16.04 LTS (64-bit)

## Bugs and issues

Please report all bugs and issues on the [tega\_teleop github issues
page](https://github.com/mitmedialab/tega_teleop/issues).

## TODO

