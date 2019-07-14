cmake_minimum_required(VERSION 2.8.3)
project(robot_navigation)

##  by hulu add for opencv compile
set(OpenCV_DIR /usr/local/share/OpenCV)
message(STATUS "    version: ${OpenCV_VERSION}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 OpenCV REQUIRED
 roscpp
 geometry_msgs
 nav_msgs
 sensor_msgs
 std_msgs
 tf
 rosbag
 message_generation
)
set (CMAKE_PREFIX_PATH ~/Qt5.7.0/5.7/gcc_64)
find_package(Qt5Widgets REQUIRED)


# Find Eigen3 (from http://wiki.ros.org/jade/Migration)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


#add_message_files(
#  FILES
#    SubmapList.msg
#    TrajectorySubmapList.msg
#    SubmapEntry.msg
#)


#generate_messages(
#  DEPENDENCIES
#    geometry_msgs
#)

catkin_package()


include_directories(include
    ${yamlcpp_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )


if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)


add_executable(main_map_server src/main_node/main_map_server.cpp
 src/map/mapreadandwrite.cpp
 src/map/map_process.cpp
 src/route_plan_algorithm/A_star.cpp
 src/map/map_deal.cpp
 )
target_link_libraries(main_map_server
 ${catkin_LIBRARIES}
 yaml-cpp
 )


add_executable(main_navigation src/main_node/main_navigation.cpp
  src/main_node/nav.cpp
  src/route_plan_algorithm/A_star.cpp
  src/map/map_deal.cpp
)
target_link_libraries(main_navigation
    ${catkin_LIBRARIES}
)

add_executable(main_Tracking src/main_node/main_Tracking.cpp
 src/main_node/TrackingPthread.cpp
 src/main_node/nav.cpp
)
target_link_libraries(main_Tracking
 ${catkin_LIBRARIES})

qt5_use_modules(main_Tracking
  Core
  Network
 )