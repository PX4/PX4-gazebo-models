cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(custom_gz_plugins)

#============================================================================
# Find gz-cmake
#============================================================================
# If you get an error at this line, you need to install gz-cmake
find_package(gz-cmake3 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

gz_configure_project(VERSION_SUFFIX)

#============================================================================
# Set project-specific options
#============================================================================

option(ENABLE_PROFILER "Enable Gazebo Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DGZ_PROFILER_ENABLE=1")
else()
  add_definitions("-DGZ_PROFILER_ENABLE=0")
endif()

if (UNIX AND NOT APPLE)
  set (EXTRA_TEST_LIB_DEPS stdc++fs)
else()
  set (EXTRA_TEST_LIB_DEPS)
endif()

#============================================================================
# Search for project-specific dependencies
#============================================================================

# Setting this policy enables using the protobuf_MODULE_COMPATIBLE
# set command in CMake versions older than 13.13
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
# This option is needed to use the PROTOBUF_GENERATE_CPP
# in case protobuf is found with the CMake config files
# It needs to be set before any find_package(...) call
# as protobuf could be find transitively by any dependency
set(protobuf_MODULE_COMPATIBLE TRUE)

gz_find_package(sdformat14 REQUIRED)
set(SDF_VER ${sdformat14_VERSION_MAJOR})

#--------------------------------------
# Find gz-plugin
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

#--------------------------------------
# Find gz-transport
gz_find_package(gz-transport13 REQUIRED COMPONENTS log parameters)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

#--------------------------------------
# Find gz-msgs
gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

set(gz-msgs_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gz_custom_msgs/
)
message(STATUS "Current directory: ${CMAKE_CURRENT_SOURCE_DIR}")
gz_find_package(gz-msgs REQUIRED)

#--------------------------------------
# Find gz-common
# Always use the profiler component to get the headers, regardless of status.
gz_find_package(gz-common5
  COMPONENTS
    profiler
    events
    av
    io
  REQUIRED
)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

#--------------------------------------
# Find gz-fuel_tools
gz_find_package(gz-fuel_tools9 REQUIRED)
set(GZ_FUEL_TOOLS_VER ${gz-fuel_tools9_VERSION_MAJOR})

#--------------------------------------
# Find gz-gui
gz_find_package(gz-gui8 REQUIRED)
set(GZ_GUI_VER ${gz-gui8_VERSION_MAJOR})
gz_find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
  PKGCONFIG "Qt5Core Qt5Quick Qt5QuickControls2")

#--------------------------------------
# Find gz-physics
gz_find_package(gz-physics7
  COMPONENTS
    heightmap
    mesh
    sdf
  REQUIRED
)
set(GZ_PHYSICS_VER ${gz-physics7_VERSION_MAJOR})

#--------------------------------------
# Find gz-sensors
gz_find_package(gz-sensors8 REQUIRED
  # component order is important
  COMPONENTS
    # non-rendering
    air_pressure
    air_speed
    altimeter
    imu
    force_torque
    logical_camera
    magnetometer
    navsat

    # rendering
    rendering
    lidar
    gpu_lidar

    # cameras
    camera
    boundingbox_camera
    segmentation_camera
    depth_camera
    rgbd_camera
    thermal_camera
    wide_angle_camera
)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

#--------------------------------------
# Find gz-rendering
gz_find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

#--------------------------------------
# Find gz-math
gz_find_package(gz-math7 REQUIRED COMPONENTS eigen3)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

#--------------------------------------
# Find gz-sim
gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

#--------------------------------------
# Find if gz command is available
find_program(GZ_TOOLS_PROGRAM gz)
if (GZ_TOOLS_PROGRAM)
  message (STATUS "Searching for gz program - found. CLI tests can be built.")
else()
  message (STATUS "Searching for gz program - not found. CLI tests are skipped.")
endif()
# Note that CLI files are installed regardless of whether the dependency is
# available during build time
set(GZ_TOOLS_VER 2)

#--------------------------------------
# Find gz-utils
gz_find_package(gz-utils2 REQUIRED COMPONENTS cli)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})

#--------------------------------------
# Find protobuf
gz_find_package(GzProtobuf
                REQUIRED
                COMPONENTS all
                PRETTY Protobuf)
set(Protobuf_IMPORT_DIRS ${gz-msgs10_INCLUDE_DIRS})


#--------------------------------------
# Find python
if (SKIP_PYBIND11)
  message(STATUS "SKIP_PYBIND11 set - disabling python bindings")
else()
  find_package(PythonLibs QUIET)
  if (NOT PYTHONLIBS_FOUND)
    GZ_BUILD_WARNING("Python is missing: Python interfaces are disabled.")
    message (STATUS "Searching for Python - not found.")
  else()
    message (STATUS "Searching for Python - found version ${PYTHONLIBS_VERSION_STRING}.")

    set(PYBIND11_PYTHON_VERSION 3)
    find_package(Python3 QUIET COMPONENTS Interpreter Development)
    find_package(pybind11 2.2 QUIET)

    if (pybind11_FOUND)
      message (STATUS "Searching for pybind11 - found version ${pybind11_VERSION}.")
    else()
      GZ_BUILD_WARNING("pybind11 is missing: Python interfaces are disabled.")
      message (STATUS "Searching for pybind11 - not found.")
    endif()
  endif()
endif()

# Plugin install dirs
#set(GZ_SIM_PLUGIN_INSTALL_DIR
#  ${CMAKE_INSTALL_PREFIX}/${GZ_LIB_INSTALL_DIR}/gz-${GZ_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
#)
#set(GZ_SIM_GUI_PLUGIN_INSTALL_DIR
#  ${CMAKE_INSTALL_PREFIX}/${GZ_LIB_INSTALL_DIR}/gz-${GZ_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins/gui
#)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(custom_ros2_msgs REQUIRED)


#============================================================================
# Create package information
#============================================================================
gz_create_packages()



#============================================================================
# Configure the build
#============================================================================
#gz_configure_build(QUIT_IF_BUILD_ERRORS)

#============================================================================
# Create package information
#============================================================================
#gz_create_packages()

include_directories(include)
include_directories(include/systems)
include_directories(include/sensors)
include_directories(include/rendering)

#include_directories(${custom_ros2_msgs_INCLUDE_DIRS})

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${rclcpp_LIBRARY_DIRS}
  #${custom_ros2_msgs_LIBRARY_DIRS}
)


set(plugins
    HelloWorld
    SampleSystem
    AirshipAerodynamics
    ModifiedMotorModel
    MulticopterMotorModelModified
    JointPositionController
    MySensor
    #AirDataBoomSensor
    FinAerodynamics
    DynamicThrustPropellerModel
    #ForceVisualization
    #UserCameraControlPlugin
)

add_library(HelloWorld SHARED src/systems/HelloWorld.cc)
add_library(SampleSystem SHARED src/systems/SampleSystem.cc src/systems/SampleSystem2.cc)
add_library(AirshipAerodynamics SHARED src/systems/AirshipAerodynamics.cc)
add_library(ModifiedMotorModel SHARED src/systems/ModifiedMotorModel.cc)
add_library(MulticopterMotorModelModified SHARED src/systems/MulticopterMotorModelModified.cc)
add_library(JointPositionController SHARED src/systems/JointPositionController.cc)
add_library(MySensor SHARED src/sensors/MySensor.cc)
add_library(MySensorSystem SHARED src/systems/MySensorSystem.cc)
#add_library(AirDataBoomSensor SHARED src/sensors/AirDataBoomSensor.cc)
#add_library(AirDataBoomSensorSystem SHARED src/systems/AirDataBoomSensorSystem.cc)
#add_library(DefaultAirspeedSensor SHARED src/sensors/DefaultAirspeedSensor.cc)
#add_library(DefaultAirspeedSensorSystem SHARED src/systems/DefaultAirspeedSensorSystem.cc)
add_library(FinAerodynamics SHARED src/systems/FinAerodynamics.cc)
add_library(DynamicThrustPropellerModel SHARED src/systems/DynamicThrustPropellerModel.cc)
#add_library(ForceVisualization SHARED src/rendering/ForceVisualization.cc)
#add_library(UserCameraControlPlugin SHARED src/rendering/UserCameraControlPlugin.cc)

# Loop through the list and link each library
foreach(plugin IN LISTS plugins)

    set_property(TARGET ${plugin} PROPERTY CXX_STANDARD 17)
    
    target_link_libraries(${plugin}

      PUBLIC
        gz-math${GZ_MATH_VER}
        gz-plugin${GZ_PLUGIN_VER}::core
        gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
        gz-common${GZ_COMMON_VER}::profiler
        gz-fuel_tools${GZ_FUEL_TOOLS_VER}::gz-fuel_tools${GZ_FUEL_TOOLS_VER}
        gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
        gz-physics${GZ_PHYSICS_VER}::core
        gz-rendering${GZ_RENDERING_VER}::core
        gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
        gz-transport${GZ_TRANSPORT_VER}::parameters
        sdformat${SDF_VER}::sdformat${SDF_VER}
        protobuf::libprotobuf
        gz-common${GZ_COMMON_VER}::graphics
        gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
      PRIVATE
        gz-plugin${GZ_PLUGIN_VER}::loader
        gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
        gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    )


endforeach()


target_link_libraries(MySensorSystem
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
  PRIVATE MySensor
)
target_include_directories(MySensorSystem
    PUBLIC src/sensors)
  

# ----------------------------------------------------------

# set_property(TARGET AirDataBoomSensor PROPERTY CXX_STANDARD 17)

# target_link_libraries(AirDataBoomSensor

#  PUBLIC
#    gz-math${GZ_MATH_VER}
#    gz-plugin${GZ_PLUGIN_VER}::core
#    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
#    gz-common${GZ_COMMON_VER}::profiler
#    gz-fuel_tools${GZ_FUEL_TOOLS_VER}::gz-fuel_tools${GZ_FUEL_TOOLS_VER}
#    gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
#    gz-physics${GZ_PHYSICS_VER}::core
#    gz-rendering${GZ_RENDERING_VER}::core
#    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
#    gz-transport${GZ_TRANSPORT_VER}::parameters
#    sdformat${SDF_VER}::sdformat${SDF_VER}
#    protobuf::libprotobuf
#    gz-common${GZ_COMMON_VER}::graphics
#    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
#    rclcpp::rclcpp
#    ${custom_ros2_msgs_LIBRARIES}
#  PRIVATE
#    gz-plugin${GZ_PLUGIN_VER}::loader
#    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
#    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}   
#)

# ----------------------------------------------------------

  
#target_link_libraries(AirDataBoomSensorSystem
#  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
#  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
#  PRIVATE gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
#  PRIVATE AirDataBoomSensor
#)
#target_include_directories(AirDataBoomSensorSystem
#    PUBLIC src/sensors)

# ----------------------------------------------------------

#set_property(TARGET DefaultAirspeedSensor PROPERTY CXX_STANDARD 17)

#target_link_libraries(DefaultAirspeedSensor

#  PUBLIC
#    gz-math${GZ_MATH_VER}
#    gz-plugin${GZ_PLUGIN_VER}::core
#    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
#    gz-common${GZ_COMMON_VER}::profiler
#    gz-fuel_tools${GZ_FUEL_TOOLS_VER}::gz-fuel_tools${GZ_FUEL_TOOLS_VER}
#    gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
#    gz-physics${GZ_PHYSICS_VER}::core
#    gz-rendering${GZ_RENDERING_VER}::core
#    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
#    gz-transport${GZ_TRANSPORT_VER}::parameters
#    sdformat${SDF_VER}::sdformat${SDF_VER}
#    protobuf::libprotobuf
#    gz-common${GZ_COMMON_VER}::graphics
#    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
#  PRIVATE
#    gz-plugin${GZ_PLUGIN_VER}::loader
#    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
#    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}   
#)

# ----------------------------------------------------------

  
#target_link_libraries(DefaultAirspeedSensorSystem
#  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
#  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
#  PRIVATE gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
#  PRIVATE DefaultAirspeedSensor
#)
#target_include_directories(DefaultAirspeedSensorSystem
#    PUBLIC src/sensors)


