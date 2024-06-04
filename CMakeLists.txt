cmake_minimum_required(VERSION 2.8.3)
project(grasp_icp)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker grasp_icp_generate_messages_cpp)

add_executable(auto_talker src/auto_talker.cpp)
target_link_libraries(auto_talker ${catkin_LIBRARIES})
add_dependencies(auto_talker grasp_icp_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener grasp_icp_generate_messages_cpp)

# Set dependent libraries

# find OpenCV
find_package(OpenCV REQUIRED)

if (OpenCV_FOUND)
    message("OpeCV FOUND:" ${OpenCV_FOUND})
    message("OpeCV VERSION:" ${OpenCV_VERSION})
    message("OpeCV Include Directories: ${OpenCV_INCLUDE_DIRS}")
    message("OpeCV Libraries: ${OpenCV_LIBS}")
else()
    message("OpenCV not found.")
endif()


# Set dependent libraries

# find Eigen
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 FOUND: ${Eigen3_FOUND}")
    message(STATUS "Eigen3 INCLUDE_DIRECTORIES: ${EIGEN3_INCLUDE_DIRS}") # it seems like you should use uppercase to output correct CMake variables even if the package name compromised lowercase characters. But you should use the origin name, i.e., the name with lowercase characters, in find_package
endif()

# find pcl
find_package(PCL REQUIRED)

if (PCL_FOUND)
    message("PCL FOUND:" ${PCL_FOUND})
    message("PCL VERSION:" ${PCL_VERSION})
    message("PCL Include Directories: ${PCL_INCLUDE_DIRS}")
    message("PCL Libraries: ${PCL_LIBS}")
else()
    message("PCL not found.")
endif()

