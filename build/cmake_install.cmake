# Install script for directory: /Users/kamesharvindsarangan/git/ROS_STAUBLI/src

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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install" TYPE PROGRAM FILES "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install" TYPE PROGRAM FILES "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install/setup.bash;/Users/kamesharvindsarangan/git/ROS_STAUBLI/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install" TYPE FILE FILES
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/catkin_generated/installspace/setup.bash"
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install/setup.sh;/Users/kamesharvindsarangan/git/ROS_STAUBLI/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install" TYPE FILE FILES
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/catkin_generated/installspace/setup.sh"
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install/setup.zsh;/Users/kamesharvindsarangan/git/ROS_STAUBLI/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install" TYPE FILE FILES
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/catkin_generated/installspace/setup.zsh"
    "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/kamesharvindsarangan/git/ROS_STAUBLI/install" TYPE FILE FILES "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/bb8/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/c3po/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/l337_ros/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/moveit_staubli/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/puma/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/staubli_resources/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/staubli_tx60_gazebo/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/staubli_tx60l_demo/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/tx_60l_moveit/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/staubli_tx2_60_support/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/staubli_tx2_90_support/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/staubli_tx60_support/cmake_install.cmake")
  include("/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/ROS-Robotics_Staubli/staubli_val3_driver/staubli_val3_driver/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/Users/kamesharvindsarangan/git/ROS_STAUBLI/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
