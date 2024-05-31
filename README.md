# Introduction

This is a project to lear robotics concepts with an simulated roboter which is based on the rosmaster X3 robot.

## Overview

This repository provides a robot description and a simulation environment as playground for robotics experiments.

One result is the simulation of a rosmaster3 robot in a gazebo simulation world.

![rviz-gazebo-simulation](docu/images/rviz-gazebo-simulation.png)

## Prerequisites

* Installed ROS 2 humble distribution
* Installed gazebo harmonic distribution

## Installation

First install required development tools

``` bash
sudo apt install python3-vcstool python3-colcon-common-extensions git wget
```

Then create a new workspace and load the git repositories which are required.

``` bash
mkdir -p ~/master3_ws/src
cd ~/master3_ws/src
wget https://raw.githubusercontent.com/cord-burmeister/master3/main/master3.yaml
vcs import < master3.yaml
```

### Install dependencies

``` bash
cd ~/master3_ws
source /opt/ros/$ROS_DISTRO/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro $ROS_DISTRO
```

### Build the project

``` bash
colcon build 
```

### Source the workspace

``` bash
. ~/master3_ws/install/setup.sh
```

## References

[What is vcstool?](https://github.com/dirk-thomas/vcstool)