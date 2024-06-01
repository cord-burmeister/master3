
# Build the description

## Overview

Gazebo is able to dynamically load models into simulation either programmatically
or through the GUI. Models exist on your computer, after they have been downloaded
or created by you. This tutorial describes Gazebo's model directory structure,
and the necessary files within a model directory.

Models in Gazebo define a physical entity with dynamic, kinematic, and
visual properties. In addition, a model may have one or more plugins,
which affect the model's behavior. A model can represent anything from a simple
shape to a complex robot; even the ground is a model.

Gazebo relies on a database to store and maintain models available
for use within simulation. The model database is a community-supported

## First model

Loading the model directly into the Simulation

``` bash
ign gazebo ~/master3_ws/src/master3/master3_description/models/master3_sdf/master3.sdf  
```

## Check the model

Check if the mecanum behavior is functional.

Open another terminal and issue a cmd_vel message to move the robot base

``` bash
ign topic -t "/model/master3_drive/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5, y: 0.5}"
```

Simulation with moving robot



## References

[Model structure and requirements](https://classic.gazebosim.org/tutorials?tut=model_structure&cat=build_robot)
