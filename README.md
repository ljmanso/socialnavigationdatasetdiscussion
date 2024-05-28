# Social Navigation Dataset Discussion
This repository contains code to record and reproduce social navigation trajectories. For now, it works with real recordings only, simulations will be added later.

To reproduce a recorded trajectory:
 - `git clone https://github.com/ljmanso/socialnavigationdatasetdiscussion`
 - `cd socialnavigationdatasetdiscussion`
 - `wget https://ljmanso.com/files/sn3_examples.tgz`
 - `tar xzf sn3_examples.tgz`
 - `sudo docker build -t sn3 .`
 - `sudo docker run -it --net=host --rm --gpus all -v $(pwd):/project -w /project sn3 bash`

That will give you a shell, but we want to connect using a VNC client. In this case we are using `xtightvncviewer`, but other clients should work just as well. The VNC server uses "sn3" as a password -feel free to change it (run the following outside the docker):
 - `xtightvncviewer localhost:5920`
 - Type the password when asked.

That will show a window manager in the container. To replay a simulated trajectory you can run the following either using an `xterm` within the virtual desktop or within the docker prompt:
 - `python3 code/sn3_replay.py FILE`

where `FILE` is any of the files in the `sn3_examples` directory. For instance:
 - `python3 code/sn3_replay.py sn3_examples/1716901588_data.pickle`

(Th)