#!/bin/bash


export DISPLAY=:20
Xvfb :20 -screen 0 1900x1000x16 &
x11vnc xxx -display :20 -N -forever &
fluxbox &
xinit &
ln -s /usr/bin/python3.8 /usr/bin/python
bash

