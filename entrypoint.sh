#!/usr/bin/env bash

export DISPLAY=:20
Xvfb :20 -screen 0 1600x950x16 &
x11vnc -passwd sn3 -display :20 -N -forever &
fluxbox &
xinit &

cd /project
bash

