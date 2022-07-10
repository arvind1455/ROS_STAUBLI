# Install script for directory: /Users/kamesharvindsarangan/git/ROS_STAUBLI/src/ROS-Robotics_Staubli/staubli_tx60_support

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Library/Developer/CommandLineTools/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/staubli_tx60_support/catkin_generated/installspace/staubli_tx60_support.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/staubli_tx60_support/cmake" TYPE FILE FILES
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/staubli_tx60_support/catkin_generated/installspace/staubli_tx60_supportConfig.cmake"
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/staubli_tx60_support/catkin_generated/installspace/staubli_tx60_supportConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/staubli_tx60_support" TYPE FILE FILES "/Users/kamesharvindsarangan/git/ROS_STAUBLI/src/ROS-Robotics_Staubli/staubli_tx60_support/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/staubli_tx60_support" TYPE DIRECTORY FILES
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/src/ROS-Robotics_Staubli/staubli_tx60_support/config"
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/src/ROS-Robotics_Staubli/staubli_tx60_support/launch"
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/src/ROS-Robotics_Staubli/staubli_tx60_support/meshes"
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/src/ROS-Robotics_Staubli/staubli_tx60_support/urdf"
    )
endif()
