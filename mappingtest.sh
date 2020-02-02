#!/usr/bin/env bash
make clean
make laptop-only
bot-lcm-tunnel 192.168.3.200 &
. setenv.sh 
./bin/botgui &
lcm-logplayer-gui data/obstacle_slam_10mx10m_5cm.log -p &
./bin/slam --mapping-only