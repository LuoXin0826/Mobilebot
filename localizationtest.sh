#!/usr/bin/env bash
# make
# make clean
pkill bot-lcm-tunnel
make laptop-only
# pkill bot-lcm-tunnel
bot-lcm-tunnel 192.168.3.200 &
. setenv.sh 
./bin/botgui &
lcm-logplayer-gui data/obstacle_slam_10mx10m_5cm.log -p &
./bin/slam data/obstacle_slam_10mx10m_5cm.map
# ./bin/slam --localization-only data/obstacle_slam_10mx10m_5cm.map