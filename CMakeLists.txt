# Copyright (C) 2011 Aldebaran Robotics
cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

# Define the name of the project
project(movementgraph)

# This include enable you to use qibuild framework
find_package(qibuild)


# Create an option to make is possible compiling the module
# as a remote executable, or as a local shared library
option(MOVEMENTGRAPH_IS_REMOTE
        "module is compiled as a remote module (ON or OFF)"
        OFF)

# Create a list of source files
set(_srcs
        moveparams.cpp
        vertex.cpp
        edge.cpp
        primalgraph.cpp
        kernelgraph.cpp
        movementgraph.cpp
        main.cpp
        )

# Create a plugin, that is a shared library, and make
# sure it is built in lib/naoqi, so that the naoqi executable
# can find it later
qi_create_lib(movementgraph SHARED ${_srcs} SUBFOLDER naoqi)

# Tell CMake that sayhelloworld depends on ALCOMMON and
# ALPROXIES.
# This will set the libraries to link sayhelloworld with,
# the include paths, and so on
qi_use_lib(movementgraph ALCOMMON ALPROXIES)