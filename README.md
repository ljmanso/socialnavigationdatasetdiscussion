# Social Navigation Dataset Discussion
This repository contains two items to support the discussion:
 - A sample JSON file.
 - A prototype for trajectory generation and rating.

Please take them as a starting point, not as a proposal.

## JSON file
We still have to agree on a file format. We may not even use JSON: [data_examle.json](https://github.com/ljmanso/socialnavigationdatasetdiscussion/blob/main/data_examle.json).

## Trajectory generation and rating prototype
Just as with the sample trajectory file, we have to agree on the tools. For this prototype, we have used [SocNavGym](https://github.com/gnns4hri/SocNavGym) on a Docker container. Because the software has a graphical interface, we use a VNC client.

First, clone the repository, build, and run the docker image:
 - `git clone https://github.com/ljmanso/socialnavigationdatasetdiscussion`
 - `cd socialnavigationdatasetdiscussion`
 - `docker build -f docker/Dockerfile -t socnav .`
 - `docker run -it -v .:/code -p 127.0.0.1:5920:5920 --rm socnav bash /code/docker/entrypoint.sh`

That will give you a shell, but we want to connect using a VNC client. In this case we are using `xtightvncviewer`, but other clients should work just as well. The VNC server uses "xxx" as a password -feel free to change it:
 - `xtightvncviewer localhost:5920`
 - Type the password when asked.

That will show a window manager with a terminal. To generate random walk trajectories, type the following in the terminal:
 - `python3 code/random_walk_agent.py`

That will generate 3 trajectories and leave the JSON files and the corresponding videos in a directory named `episode_recordings`. To rate them, once the previous script has finished, run:
 - `python3 code/rate.py`

Once all the trajectories are rated, the output will be written in `ratings.json`.
