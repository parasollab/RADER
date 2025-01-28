# Adding an Analytical IK Solver to RADER

For a fast analytical IK solver, you can add [IKFast](https://moveit.picknik.ai/humble/doc/examples/ikfast/ikfast_tutorial.html) solvers to Unity. These solvers are inherently specific to the robot they were generated for, so you will need to generate the necessary files to support a new robot.

## Generating a Solver

Depending on the robot, you may need to generate your own solver using OpenRAVE or you may be able to find a GitHub repo where someone has already generated one. For example, [here](https://github.com/cambel/ur_ikfast) is a repo containing IKFast solvers for UR robots.

## Compiling Unity Plugin Files

Unity allows you to run C/C++ code through [Native Plugins](https://docs.unity3d.com/6000.0/Documentation/Manual/plug-ins-native.html). You will need to compile the code differently for each platform you intend to use it on. For example, if I want to use the play button in the editor on my mac, I will have to compile for mac. Most VR headsets use Android. We recommend compiling for every platform just in case.

### Writing a Wrapper

Depending on where you get your solver from, you may need to write a C++ wrapper from which you can call the inverse kinematics function. This is because you cannot make an object of type `Kinematics` from the C++ code from which to call the IK function in C#.

Here is an example of a wrapper:

`unity_wrapper.h`:
```cpp
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Forward declare (opaque pointer).
// We never expose the actual C++ class to C#;
// just pass it around by pointer.
typedef struct KinematicsWrapper KinematicsWrapper;

// Create / destroy
KinematicsWrapper* create_kinematics();
void destroy_kinematics(KinematicsWrapper* kin);

// For forward kinematics
// We pass raw pointers to floats rather than std::vector.
// We'll also need the size of the array (joint_config_length).
// The result is written into an output array.
void forward_kinematics(
    KinematicsWrapper* kin,
    const float* joint_config,
    int joint_config_length,
    float* out_array,
    int out_array_length);

// For inverse kinematics
// Similar approach.
void inverse_kinematics(
    KinematicsWrapper* kin,
    const float* ee_pose,
    int ee_pose_length,
    float* out_array,
    int out_array_length);

#ifdef __cplusplus
}
#endif
```

`unity_wrapper.cpp`:
```cpp
#include "include/unity_wrapper.h"
#include "include/Kinematics.hpp" // your original header
#include <vector>
#include <cstring>

// We define a small wrapper struct that contains
// the actual C++ class pointer.
struct KinematicsWrapper {
    robots::Kinematics* impl;
};

extern "C" {

// Create
KinematicsWrapper* create_kinematics()
{
    auto wrapper = new KinematicsWrapper();
    wrapper->impl = new robots::Kinematics();
    return wrapper;
}

// Destroy
void destroy_kinematics(KinematicsWrapper* kin)
{
    if (!kin) return;
    delete kin->impl;
    delete kin;
}

// forward_kinematics
void forward_kinematics(
    KinematicsWrapper* kin,
    const float* joint_config,
    int joint_config_length,
    float* out_array,
    int out_array_length)
{
    if (!kin) return;

    // Convert the incoming float pointer to a std::vector<float>
    std::vector<float> inputVec(joint_config, joint_config + joint_config_length);

    // Call the actual C++ method
    std::vector<float> result = kin->impl->forward(inputVec);

    // Copy result into out_array, up to out_array_length
    int count = std::min((int)result.size(), out_array_length);
    for (int i = 0; i < count; i++) {
        out_array[i] = result[i];
    }
}

// inverse_kinematics
void inverse_kinematics(
    KinematicsWrapper* kin,
    const float* ee_pose,
    int ee_pose_length,
    float* out_array,
    int out_array_length)
{
    if (!kin) return;

    std::vector<float> inputVec(ee_pose, ee_pose + ee_pose_length);
    std::vector<float> result = kin->impl->inverse(inputVec);

    int count = std::min((int)result.size(), out_array_length);
    for (int i = 0; i < count; i++) {
        out_array[i] = result[i];
    }
}

} // extern "C"
```

### Compiling for Mac

You will need to use XCode to compile a `.bundle` file. Create a new **Bundle** project and add your C++ files there. The **Target Membership** controls which files get compiled. Add your cpp files, likely with the exception of your file named something like `<robot>_ikfast<version>.cpp` since this is likely imported into another cpp file and you don't want it included twice.

Make sure you add `liblapack` to the **Frameworks and Libraries** section of the project configuration page unless you've included it some other way.

Build the project and add it to the `Plugins/osx` directory of RADER.

### Compiling for Windows

You may need to compile LAPACK from source for Windows. 
