# docker build -f docker/Dockerfile -t socnav .
# docker run -it -v .:/code -p 127.0.0.1:5920:5920 --rm socnav bash /code/docker/entrypoint.sh

#
# 3. See results:
# tensorboard --logdir ~/logdir
# System
FROM ubuntu:jammy
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/London

RUN apt-get update
RUN apt-get install -y ffmpeg git python3-pip vim libglew-dev x11-xserver-utils xvfb
RUN apt-get install -y cmake python3-pyside2* x11vnc xvfb fluxbox
RUN apt-get install -y terminator mc
RUN apt-get clean


RUN pip3 install opencv-python-headless --force
RUN pip3 install --upgrade pip
RUN pip3 install tensorflow-cpu numpy 
RUN pip3 install dgl -f https://data.dgl.ai/wheels/repo.html
RUN pip3 install Cython
RUN pip3 install git+https://github.com/sybrenstuvel/Python-RVO2/
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install pygame PySide2 PyQt5 pyqtgraph
RUN pip3 install PyYAML


RUN git clone https://github.com/gnns4hri/SocNavGym -b pygame /SocNavGym
RUN pip3 install /SocNavGym
RUN rm -rf /usr/local/lib/python3.10/dist-packages/cv2/qt



# X stuff
EXPOSE 5920
ENV DISPLAY :20
RUN echo "exec fluxbox" >> ~/.xinitrc
RUN chmod +x ~/.xinitrc
RUN mkdir ~/.fluxbox
RUN echo "[startup] {terminator -e 'cd /code && /bin/bash'}" > ~/.fluxbox/apps

# RUN apt install -y libpulse-mainloop-glib0 libqt5multimedia5-plugins gstreamer1.0-qt5 libqt5multimediagsttools5 libqt5gstreamerutils-1.0-0 libqt5gstreamerui-1.0-0 libqt5gstreamer-1.0-0 gstreamer1.0-plugins-bad gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly phonon4qt5-backend-gstreamer qml-module-qtgstreamer qtgstreamer-plugins-qt5 
RUN apt install -y libqt5multimedia5-plugins gstreamer1.0-qt5 gstreamer1.0-plugins-bad gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly phonon4qt5-backend-gstreamer qml-module-qtgstreamer qtgstreamer-plugins-qt5 
RUN apt install -y libqt5multimediagsttools5 libqt5gstreamerutils-1.0-0 libqt5gstreamerui-1.0-0 libqt5gstreamer-1.0-0 gstreamer1.0-plugins-bad gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly phonon4qt5-backend-gstreamer qml-module-qtgstreamer qtgstreamer-plugins-qt5 libgstreamer-opencv1.0-0 gstreamer1.0-qt5 libgtk-4-media-gstreamer libgstreamermm-1.0-1 libextractor-plugin-gstreamer libcanberra-gstreamer gstreamer1.0-tools  gstreamer1.0-qt5   gstreamer1.0-python3-plugin-loader gstreamer1.0-pulseaudio gstreamer1.0-plugins-base-apps gstreamer1.0-plugins-bad-apps gstreamer1.0-nice   gstreamer1.0-libav     gstreamer1.0-gtk3     gstreamer1.0-alsa
