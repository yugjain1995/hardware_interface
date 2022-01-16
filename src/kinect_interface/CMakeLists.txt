CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

# Set project name
PROJECT(kinect_interface VERSION 1.0)

# Set frenect2 library path variable
set(freenect2_DIR $ENV{HOME}/freenect2/lib/cmake/freenect2)

# find packages
set(DEPENDENCIES image_transport freenect2 OpenCV sensor_msgs cv_bridge)
macro(find_packages)
  foreach(package ${ARGN})
    find_package(${package} REQUIRED)
  endforeach()
endmacro()
find_packages(${DEPENDENCIES})
find_package(Boost COMPONENTS log) 

# Add executable
add_executable(kinect_interface kinect_interface.cpp)

# Include
set(KINECT_INTERFACE_INCLUDE ${image_transport_INCLUDE_DIRS} ${freenect2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${cv_bridge_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
macro(includes)
  foreach(package ${ARGN})
    target_include_directories(kinect_interface PUBLIC ${package})
  endforeach()
endmacro()
includes(${KINECT_INTERFACE_INCLUDE})

# Libraries
set(KINECT_INTERFACE_LIBRARIES ${image_transport_LIBRARIES} ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${sensor_msgs_LIBRARIES} ${cv_bridge_LIBRARIES} ${Boost_LIBRARIES})
macro(libs)
  foreach(package ${ARGN})
    target_link_libraries(kinect_interface ${package})
  endforeach()
endmacro()
libs(${KINECT_INTERFACE_LIBRARIES})