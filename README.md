## About
This repository contains algorithms developed for the student paper proposed for Rector's award at University of Zagreb.

**Title**: Decentralized formation control for a multi-agent system of autonomous spherical robots

**Authors**: Antonella Barišić, Marko Križmančić

**Abstract**:
In this work, a decentralized control algorithm based on Reynolds' rules is implemented on a multi-agent system of spherical robots. The algorithm procedurally generates motion patterns that resemble those characteristic for flocks of birds or schools of fish. Generated motion patterns allow robots to move in a closed space with static obstacles. A Bluetooth driver for controlling the robots has also been developed. The complete system is implemented using the ROS framework and Python programming language. The results of this work are demonstrated with experiments in a simulated environment, as well as in the real world using Sphero SPRK+ robots localized with _OptiTrack_.

## IMPORTANT NOTICE:
I recently started a reorganization of the repository and the current version is be broken (unavailable outside packages are necessary). Unfortunately, I can't say when that might get fixed. In the meantime, check out an older version when everything was still in the same place: https://github.com/mkrizmancic/sphero_formation/tree/0ac14aad3dd1a0af26f191c017e213279eebd52e

## Installation
Simply clone this repository inside ROS workspace and run `catkin build` in workspace root.

Simulation part uses [stage_ros](http://wiki.ros.org/stage_ros) simulator.

If you wish to use this with Sphero robots, you must also download this repository: https://github.com/antonellabarisic/sphero_sprk_ros.

_OptiTrack_ is used for localization of robots. However, Spheros cannot be equipped with tracking markers, so only their LEDs are used for tracking. Each Sphero shows up as a single marker. In order to stream positions of single markers, you must also download this repository: https://github.com/mkrizmancic/mocap_optitrack/tree/new-and-old-support-updated.

## Usage
**Simulation:**
1. Set values for all desired parameters inside _launch/setup_sim.launch_.
1. Set initial velocities in _cfg/sphero_init_vel.cfg_.
1. In first terminal run `roscore` (optional)
1. In second terminal run `roslaunch sphero_formation setup_sim.launch`
1. In third terminal run `roslaunch sphero_formation reynolds_sim.launch`

**Real world:**
1. Set number of robots used in _launch/drivers.launch_. <br>
(First _n_ robots defined with MAC addresses in _sphero_addresses.txt_ will be used.)
1. Set values for all other desired parameters inside _launch/setup_real.launch_.
1. Set initial velocities in _cfg/sphero_init_vel.cfg_.
1. In first terminal run `roscore` (optional)
1. In second terminal run `roslaunch sphero_formation drivers.launch`
1. In third terminal run `roslaunch sphero_formation setup_real.launch`
1. In fourth terminal run `roslaunch sphero_formation tracking.launch`
1. In fifth terminal run `roslaunch sphero_formation flocking.launch`

Flocking algorithm parameters can be changed during runtime with _rqt_reconfigure_. GUI for changing parameters is launched from `setup_*.launch` files. 

##### Manual control:
Robots can be controlled manually with Logitech F710 joystick.

TODO: specify button mappings


