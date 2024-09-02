# Robot Action Demonstration in Extended Reality (RADER)

This repository contains the Unity package for the system presented in (TODO cite paper).
It allows the user to load a robot, specified with a URDF file, into Unity and interact with it to provide trajectory demonstrations by interacting with the robot by directly pushing or dragging joints, using sliders, or moving a target end effector position.
We intend for this to be used for robotic learning from demonstration, however, you may find it useful for a variety of applications.

## Package Installation

This package can be installed using Unity's Package Manager window. Add the package via git and use this URL: `https://github.com/parasollab/RADER`.

This package can be used on its own but is maximally useful when used in conjunction with ROS nodes running on an external computer to process the trajectory data collected. Our ROS nodes can be found here: [https://github.com/parasollab/hri_ws](https://github.com/parasollab/hri_ws).

## Usage

We provide an example Unity project that uses this package here: [https://github.com/parasollab/VR_Robot](https://github.com/parasollab/VR_Robot).

The `VR_Robot` repository linked above also has a wiki with documentation describing the usage of this package and the example project.
