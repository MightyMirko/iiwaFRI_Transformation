# Project Title

The project is a robotics-related development that involves the utilization of the FRI (Fast Research Interface)
Client SDK (Software Development Kit) in C++. The primary focus is on robot control and communication with a KUKA
robotic arm. The project includes a collection of source code, configuration files, and documentation to facilitate
the integration and understanding of the FRI Client SDK.

## Overview

### Key Components:

- **FRI Client SDK (C++)**
  - The project includes the FRI Client SDK in C++.
  - The SDK facilitates communication and control of KUKA robotic arms.
  - Loosely Based on:
    - [@lbr-stack/fri](https://github.com/lbr-stack/fri)
    - [https://github.com/lbr-stack/fri](https://github.com/lbr-stack/fri)

- **Robotics Libraries**
  - Various XML files describe robot models, such as "iiwa_dh_model.xml" and "iiwa_my.xml."
  - These models likely represent different configurations or kinematic structures of the KUKA robotic arm.
  - Based on:
    - [@roboticslibrary/rl](https://github.com/roboticslibrary/rl)
    - [https://github.com/roboticslibrary/rl](https://github.com/roboticslibrary/rl)

- **Application Code**
  - The "masters" directory contains application-specific code, including "mastersApp.cpp" and "mastersclient.cpp."
  - These files are crucial for interfacing with the FRI Client SDK and implementing specific functionalities.

- **Documentation**
  - The "img" directory contains images used for documentation purposes.
  - The "README.md" file provides essential information, instructions, and details about the project.

## Getting Started

### Development Environment:

The project uses CMake for building and managing the build process. Dependencies may include specific libraries or tools required for successful execution.

```bash
#!/bin/bash

# Set your project directory
PROJECT_DIR="/home/mirko/CLionProjects/thesis2024_orphaned"

# Set the build directory
BUILD_DIR="${PROJECT_DIR}/build"

# Set additional CMake options
CMAKE_OPTIONS="-DCMAKE_BUILD_TYPE=Debug -DBUILD_FRI_APPS=ON -DBUILD_MASTERS=ON -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_DIR}"

# Run CMake
cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}" ${CMAKE_OPTIONS}
```

```bash
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_FRI_APPS=ON -DBUILD_MASTERS=ON -B build
```
