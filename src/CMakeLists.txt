CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

# Set project name
PROJECT(kinect_interface VERSION 1.0)

# find and load packages
find_package(freenect2 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(kinect_interface_tester kinect_interface_tester.cpp kinect_interface.cpp viewer.cpp flextGL.cpp)
target_include_directories(kinect_interface_tester PUBLIC . PUBLIC ${freenect2_INCLUDE_DIR})
target_link_libraries(kinect_interface_tester ${freenect2_LIBRARIES} glfw OpenGL::GL)