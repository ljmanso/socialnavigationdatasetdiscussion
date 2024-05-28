##############################################################################################################
# To build the Docker container
# sudo docker build -t ros2docker .
#
# To run it:
# sudo docker run -it --net=host --rm --gpus all -v $(pwd):/project -w /project ros2docker bash
#
# To connect to the X server:
# xtightvncviewer localhost:5920
#
##############################################################################################################

FROM ros:humble-ros-base-jammy

# install ros2 packages
RUN apt-get update && apt-get install -y ros-humble-desktop=0.10.0-1* && rm -rf /var/lib/apt/lists/*

RUN apt-get update

RUN apt-get install -y python3-pip libbz2-dev 

RUN python3 -m pip install zeroc-ice


RUN apt-get install -y mc
RUN apt-get clean

# VNC server
EXPOSE 5920
ENV DISPLAY :20
RUN echo "exec fluxbox" >> ~/.xinitrc
RUN chmod +x ~/.xinitrc
RUN mkdir ~/.fluxbox
RUN echo "[startup] {xterm -e 'cd /code && /bin/bash'}" >> ~/.fluxbox/apps
RUN echo "[startup] {xterm -e 'cd /code && /bin/bash'}" >> ~/.fluxbox/apps
RUN echo "[exec] (xterm) {xterm -e 'cd /code && /bin/bash'}" >> ~/.fluxbox/menu

RUN apt-get install -y terminator firefox
RUN pip3 install wandb
RUN pip3 install opencv-python
# RUN sudo apt-get install -y python3-sphinx python3-pip
# RUN sudo pip3 install sphinx_autodoc_typehints
# RUN sudo python3 -m pip install sphinx_rtd_theme

#RUN /bridge/install_rclpy.sh

## Environment setup
#/RUN echo '#!/usr/bin/env bash' > /ros_entrypoint.sh
#RUN echo 'source /opt/ros/humble/setup.bash' >> /ros_entrypoint.sh
#RUN echo 'exec "$@"' >> /ros_entrypoint.sh
#RUN chmod +x /ros_entrypoint.sh
#

RUN apt-get install -y ffmpeg git python3-pip vim libglew-dev x11-xserver-utils xvfb
RUN apt-get install -y cmake python3-pyside2* x11vnc xvfb fluxbox



RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

ENTRYPOINT ["/project/entrypoint.sh"]
## Run bash
CMD ["bash"]

