# Robot Action Demonstration in Extended Reality (RADER)

This repository contains the Unity package for the system presented in (TODO cite paper).
It allows the user to load a robot, specified with a URDF file, into Unity and interact with it to provide trajectory demonstrations by interacting with the robot by directly pushing or dragging joints, using sliders, or moving a target end effector position.
We intend for this to be used for robotic learning from demonstration, however, you may find it useful for a variety of applications.

**Important Note: version 2 (main branch) is under development. Those wanting to use a stable release version should use version 1.0 (see releases).** To import this from Unity's package manager, use this git URL: `https://github.com/parasollab/RADER.git#v1.0`

## VERSION 2 - PATCH NOTES (Jan 12, 2025)

RADER is undergoing a sizeable refactor. Here are the main affected components:

1) The introduction of a new script, `RobotManager`. **This script must be attached to each robot GameObject.** It functions as a black box to access robot functionality such as getting and setting joint angles without needing to touch knobs. The intention is to reduce complexity and make adding new scripts that need to access this functionality easier.

2) The modularization of inverse kinematics. There is now a base class called `IKSolver` that each implemented inverse kinematics solver must inherit from. The purpose of this is to allow us to use IK solvers made for specific robots, which can be faster and more accurate. We still have `CCDIK` as a fallback. The user can set the IK solver they want to use in the `RobotManager` component attached to the robot.

3) The decoupling of target tracking from the robot model. Previously, the target sphere was attached to the end of the robot. Now, the target prefab should be added to the scene. Then, the `TargetSphere` script should be added as a component to an empty game object in the scene, and the target and robot fields populated. The purpose of this decoupling is that the user may want to do something else with the robot other than follow the sphere (e.g. hand tracking).

4) The introduction of hand tracking. This is part of an effort to enable demonstrating bimanual manipulation. This is best seen in the `AR_RADER` repo/project. The `HandMirror` script should be attached to an empty game object in the scene and populated to be used.

5) The refactoring of `SetupUI` to use `RobotManager`. Instead of populating `SetupUI` from the URDF processing script, it is now a separate component. The purpose of this is for code clarity/ease of use, as well as setting the foundation to accommodate controlling multiple robots with one UI menu. The `SetupUI` script should be added to an empty game object in the scene and populated to be used. This script now takes a list of robot GameObjects, but currently only the first is used. We will need to update it to account for multiple robots.

6) To support mimic joints (used by 2-finger grippers), we have made a fork of Unity's URDF importer. This has been added as a submodule to both `VR_Robot` and `AR_RADER`. Make sure you run `git submodule update --init` after you pull the new changes.

## Package Installation

To import RADER into an existing project, use the following instructions. Alternatively, we provide an example project here: [https://github.com/parasollab/VR_Robot](https://github.com/parasollab/VR_Robot).


### Dependencies

Install these dependencies via Unity's Package Manager window:

```bash
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector#v0.7.0

https://github.com/parasollab/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

This package can be installed using Unity's Package Manager window. Add the package via git and use this URL: `https://github.com/parasollab/RADER.git#v1.0`.

This package can be used on its own but is maximally useful when used in conjunction with ROS nodes running on an external computer to process the trajectory data collected. Our ROS nodes can be found here: [https://github.com/parasollab/hri_ws](https://github.com/parasollab/hri_ws).
