# Assignment: PA0 - Random Walk

#### Author: Megan Liu

#### Class: COSC81.01 Principles of Robot Design and Programming

#### Instructor: Alberto Quattrini Li

#### Term: Spring 2025 (4/4/2025)

#### Description: This code implements a random walk behavior for a robot using ROS2. The robot moves forward until it detects an obstacle within a certain distance, at which point it rotates a random angle [-pi, pi] before continuing to move forward.

## How to run:

1. Start docker container in terminal via the command

```
docker compose up
```

If you aren't setup yet, follow the instructions here: https://canvas.dartmouth.edu/courses/71483/pages/instructions-for-setting-up-ros-2-docker

You should see success messages similar to the following.

```
 ⠿ Container vnc-ros-ros-1    Cr...                           0.0s
 ⠿ Container vnc-ros-novnc-1  Recreated                       0.1s
Attaching to vnc-ros-novnc-1, vnc-ros-ros-1
... (comment: more messages, with the latest being in the current version of Docker)

vnc-ros-novnc-1  | 2023-03-29 19:45:10,919 INFO success: xterm entered RUNNING state, process has stayed up for > than 1 seconds (startsecs)
```

2. Connect to the simulator by opening localhost:8080/vnc.html in the browser

You should see a robot in a simulated environment.

3. In another terminal, open terminal in the simulator via the command

```
docker compose exec ros bash
```

4.  In the terminal in step 3, run the random walk program via the command

```
python3 pa0_random_walk.py
```

The robot should start walking randomly (ie walking straight until it's too close to an obstacle, in which case it rotates then walks straight).
