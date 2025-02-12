# Adding an Analytical IK Solver to RADER

For a fast analytical IK solver, you can add [IKFast](https://moveit.picknik.ai/humble/doc/examples/ikfast/ikfast_tutorial.html) solvers to Unity. These solvers are inherently specific to the robot they were generated for, so you will need to generate the necessary files to support a new robot.

**Note: there are a lot of steps to this and therefore this cannot be an exhaustive guide. You may need to consult other resources along the way if you run into problems or confusion.**

## Unity C# Script

You will need a Unity script to call the solver. See `UR5eAnalyticalIK.cs` as an example. Each function from IKFast that you want to call will need to be marked using `DLLImport(<name>)`.

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

You may need to compile LAPACK from source for Windows. You can download the latest version from [netlib](https://www.netlib.org/lapack/index.html#_lapack_version_3_12_1). The easiest way to build it is to download CMake and build the library from CMake's GUI. You will need a Fortran compiler. If you do not have one, you can download and install the [Intel Fortran Compiler](https://www.intel.com/content/www/us/en/developer/tools/oneapi/fortran-compiler.html). In the CMake GUI, you may need to go into **Configure** and set the location of the Fortran compiler. You can find its location by searching for an executable file named `ifx` in the file explorer. You may also need to specify `fortran=ifx` in the optional toolset. Make sure to check the box to build LAPACKE as well, this is the C interface.

Use CMake to generate a Visual Studio solution and open it in Visual Studio. Build the **ALL_BUILD** target and the **INSTALL** target (you may need to run Visual Studio as an administrator to do this).

Once LAPACK is built, create a new DLL Visual Studio project and add your cpp files. You will need to go into the project properties and add:
1) In the linker, an additional library directory that points to the LAPACK lib directory (should be under Program Files (x86?) / LAPACK / lib (or similar).
2) Also in the linker, add `libblas; liblapack; liblapacke` to the additional libraries.
3) In the C/C++ options, add an additional include directory that points to the LAPACK include directory (should be close to the lib directory).

You may need to edit the `<robot>_ikfast<version>.cpp` file to use LAPACKE explicitly by including `lapacke.h`. You may need to change function names to account for this. You may also need to exclude this file from the build as with the Mac compilation.

Build the visual studio project and place the resulting dll file into Unity.

### Compiling for Android

Download Android Studio and create a new Native C++ project. Set the Android SDK version based on the version used by your Android device. For example, the Meta Quest 3 uses at minimum SDK version 33. Add your cpp files to the cpp directory. Due to a lack of a suitable Fortran compiler, you will need to use CLAPACK here, which is a version of LAPACK that was translated from Fortran to C. You can download the zip file [here](https://www.netlib.org/clapack/) and place the files into your Android Studio project in a directory like `cpp\third_party\clapack`

You will need to add a `CMakeLists.txt` file to compile CLAPACK. It should look something like this:
```cmake
cmake_minimum_required(VERSION 3.10)

project(clapack C)

# Gather all source files in BLAS, SRC, F2CLIBS, etc.
# You might do something like:
file(GLOB CLAPACK_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/BLAS/SRC/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/SRC/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/INSTALL/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/F2CLIBS/libf2c/*.c
        # etc.
)

add_library(clapack STATIC ${CLAPACK_SRC})

# Include the headers for CLAPACK
target_include_directories(clapack
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/INCLUDE
        ${CMAKE_CURRENT_SOURCE_DIR}/F2CLIBS/libf2c
        # etc.
)

# Optionally set any compiler flags needed. For example, CLAPACK might need special flags:
target_compile_options(clapack PRIVATE -DF2C_COMPATIBLE)
# (Check CLAPACK docs or netlib forums for recommended flags)
```

Your main `CMakeLists.txt` should look something like this (edit to reflect your file names):
```cmake
# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html.
# For more examples on how to use CMake, see https://github.com/android/ndk-samples.

# Sets the minimum CMake version required for this project.
cmake_minimum_required(VERSION 3.22.1)

# Declares the project name. The project name can be accessed via ${ PROJECT_NAME},
# Since this is the top level CMakeLists.txt, the project name is also accessible
# with ${CMAKE_PROJECT_NAME} (both CMake variables are in-sync within the top level
# build script scope).
project("<robot>_ikfast")

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -- Build CLAPACK (example), either by subdirectory or direct add_library --
# If you have a separate CMakeLists in clapack/, you can do:
add_subdirectory(third_party/clapack)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.
#
# In this top level CMakeLists.txt, ${CMAKE_PROJECT_NAME} is used to define
# the target library name; in the sub-module's CMakeLists.txt, ${PROJECT_NAME}
# is preferred for the same purpose.
#
# In order to load a library into your app from Java/Kotlin, you must call
# System.loadLibrary() and pass the name of the library defined here;
# for GameActivity/NativeActivity derived applications, the same library name must be
# used in the AndroidManifest.xml file.
#add_library(${CMAKE_PROJECT_NAME} SHARED
#        # List C/C++ source files with relative paths to this CMakeLists.txt.
#        native-lib.cpp)
add_library(${CMAKE_PROJECT_NAME}
        SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/ikfast_wrapper.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/unity_wrapper.cpp
)

# Include any directories that contain header files
target_include_directories(ur5e_ikfast
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Specifies libraries CMake should link to your target library. You
# can link libraries from various origins, such as libraries defined in this
# build script, prebuilt third-party libraries, or Android system libraries.
target_link_libraries(${CMAKE_PROJECT_NAME}
        # List libraries link to the target library
        clapack
        android
        log)
```

You may need to edit some parts of CLAPACK to get it to compile for Android. When the project builds, it will produce several `.so` files, one per architecture supported by Android. Copy each of these files into Unity and in the inspector, make sure that each is set to the correct architecture and is marked as *Executable* rather than *Symbol*.

### Compiling for Linux

Compiling for Linux is more simple than the other platforms. Install cmake, LAPACK, and BLAS using `sudo apt install cmake liblapack-dev libblas-dev`. Then, create a `CMakeLists.txt` file in your directory that looks something like this (fill in <robot>):

```cmake
cmake_minimum_required(VERSION 3.10)
project(<robot>_ikfast)

# Build a shared library (.so on Linux)
add_library(<robot>_ikfast SHARED
    ikfast_wrapper.cpp
    unity_wrapper.cpp
)

# If you use additional libraries, link them here:
# Link system LAPACK and BLAS
target_link_libraries(<robot>_ikfast
    PRIVATE
    lapack
    blas
)

# Set compile options as needed
target_compile_features(<robot>_ikfast PRIVATE cxx_std_17)
```

Build the library using:

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

Then, copy the resulting `.so` file into `RADER\Runtime\Plugins\linux` and set the plugin properties.
