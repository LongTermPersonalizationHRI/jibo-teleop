#!/bin/bash
echo $1, $2, $3, $4

#check $1
list="p00 p01 p02 p03 p04 p05 p06 p07 p08 p09 p10 p11 p12 p13 p14 p15 p16 p17 p18 p19 p20 p21 p22 p23 p24 p25 p26 p27 p28 p29 p30 p31 p32 p33 p34 p35 p36 p37 p38 p39 p40"

if ! [[ $list =~ (^| )$1($| ) ]]; then
  echo "error: participant_id [$1] does not exist"
  echo "Usage: ./start_gfta_collection.sh <participant_id> <experimenter_name> <record_option>"
  echo "./start_gfta_collection.sh p00 sam rosbag-record"
  exit
fi

#check $2
list="sam gary amber alejandra"
if ! [[ $list =~ (^| )$2($| ) ]]; then
  echo "error: experimenter [$2] does not exist"
  echo "Usage: ./start_gfta_collection.sh <participant_id> <experimenter_name> <record_option>"
  echo "./start_gfta_collection.sh p00 sam rosbag-record"
  exit
fi

#check $3
list="rosbag-record no-rosbag"
if ! [[ $list =~ (^| )$3($| ) ]]; then
  echo "error: record option [$3] does not exist"
  echo "Usage: ./start_gfta_collection.sh <participant_id> <experimenter_name> <record_option>"
  echo "./start_gfta_collection.sh p00 sam rosbag-record"
  exit
fi

mkdir -p log
mkdir -p rosbag


#gnome-terminal --geometry 40x120+0+0 -e "python ../src/jibo-teleop.py $1 $2 $3"
echo "Make sure you are running from top level of directory (e.g. $ ./launch/start_gfta_collection.sh)"
python ./src/jibo_teleop.py $1 $2

if [ $3 = 'record' ]; then
  ./scripts/rosbag_record.sh $1 $2 $3
fi
