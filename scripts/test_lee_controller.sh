#!/bin/bash
GAZEBO_VER_LINE=$(gazebo -v)
GAZEBO_VER=$(awk -v RS=[0-9]+ '{print RT+0;exit}' <<< $GAZEBO_VER_LINE)
export GAZEBO_MAJOR_VERSION=$GAZEBO_VER

roslaunch dji_m100_gazebo test_lee_controller.launch
