cmake_minimum_required (VERSION 2.8.12)

#set (CMAKE_PREFIX_PATH /opt/Qt5.6.1/5.6/gcc_64/lib/cmake)

project (RNE_dehazing)

add_definitions(-std=c++11)

# Find includes in corresponding build directories
# Adds in each directory ${CMAKE_CURRENT_SOURCE_DIR} and ${CMAKE_CURRENT_BINARY_DIR}
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
# Instruct CMake to run moc automatically when needed.


# Find the Qt libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    image_transport
    cv_bridge
    dynamic_reconfigure
)


set (SRC
    src/main.cpp
    src/rosthread.cpp
)

generate_dynamic_reconfigure_options(
    cfg/RNE_dehazing.cfg
  #...
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge dynamic_reconfigure
    DEPENDS OpenCV
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${SRC_DIR}
)


# add UI file

add_executable (RNE_dehazing ${SRC})
target_link_libraries (RNE_dehazing
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
