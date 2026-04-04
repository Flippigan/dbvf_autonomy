# Install script for directory: /home/finn/Documents/ardu_ws/src/dbvf_autonomy

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/install/dbvf_autonomy")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy/environment" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy/environment" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dbvf_autonomy-0.0.1-py3.10.egg-info" TYPE DIRECTORY FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_python/dbvf_autonomy/dbvf_autonomy.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/dbvf_autonomy" TYPE DIRECTORY FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/dbvf_autonomy/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/install/dbvf_autonomy/local/lib/python3.10/dist-packages/dbvf_autonomy"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy" TYPE DIRECTORY FILES
    "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/launch"
    "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/config"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dbvf_autonomy" TYPE PROGRAM FILES
    "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/scripts/mavlink_interface_node"
    "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/scripts/tag_detector_adapter_node"
    "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/scripts/precision_landing_node"
    "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/scripts/tag_visualizer_node"
    "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/scripts/mission_sequencer_node"
    "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/scripts/arduino_interface_node"
    "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/scripts/csi_camera_node"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/dbvf_autonomy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/dbvf_autonomy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy/environment" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy/environment" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_index/share/ament_index/resource_index/packages/dbvf_autonomy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy/cmake" TYPE FILE FILES
    "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_core/dbvf_autonomyConfig.cmake"
    "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/ament_cmake_core/dbvf_autonomyConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbvf_autonomy" TYPE FILE FILES "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/finn/Documents/ardu_ws/src/dbvf_autonomy/build/dbvf_autonomy/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
