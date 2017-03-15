cmake_minimum_required(VERSION 2.8.3)
project(multi_robot_workstation)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    roscpp
    rosconsole
    message_filters
    sensor_msgs
    cv_bridge
    std_msgs
    image_transport
    visualization_msgs
    message_generation
    multi_robot_slam
)
set(OpenCV_DIR /usr/local/opencv2.4.9/share/OpenCV)
#find_package(OpenCV 2.4.9 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
#find_package(Eigen REQUIRED)
message("OPECV_INCLUDE:"${OpenCV_INCLUDE_DIRS}\n)

#include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
)

add_executable(multiRobWorkstation src/multiRobWorkstation.cpp)
message("OPECV_LIBRARIES:"${OpenCV_LIBRARIES}\n)
target_link_libraries(multiRobWorkstation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#install(TARGETS imageConverter
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
