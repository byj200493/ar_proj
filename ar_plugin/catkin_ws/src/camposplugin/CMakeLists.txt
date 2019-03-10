cmake_minimum_required(VERSION 2.4.6)
project(rosbagreadposeplugin)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#find_package(OpenCV REQUIRED)


#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include_directories(
  include/header
)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(catkin REQUIRED COMPONENTS rosbag)
find_package(OpenCV REQUIRED HINTS /usr/lib/x86_64-linux-gnu/ INCLUDE_DIRECTORIES(/usr/include/opencv2))
find_package(roscpp REQUIRED)
find_package(roslib REQUIRED)
find_package(catkin REQUIRED COMPONENTS nav_msgs geometry_msgs visualization_msgs image_transport eigen_conversions pcl_ros)

set(HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/header/plugin.h /opt/ros/indigo/include/rosbag/bag.h /opt/ros/indigo/include/rosbag/view.h /opt/ros/indigo/include/std_msgs/Int32.h /opt/ros/indigo/include/std_msgs/String.h /usr/include/boost/foreach.hpp /opt/ros/indigo/include/sensor_msgs/CameraInfo.h /opt/ros/indigo/include/image_transport/image_transport.h "/usr/local/include/pcl-1.8/pcl/ModelCoefficients.h" /usr/local/include/pcl-1.8/pcl/io/pcd_io.h /usr/local/include/pcl-1.8/pcl/point_types.h /usr/local/include/pcl-1.8/pcl/sample_consensus/method_types.h /usr/local/include/pcl-1.8/pcl/sample_consensus/model_types.h /usr/local/include/pcl-1.8/pcl/segmentation/sac_segmentation.h /usr/include/eigen3/Eigen/StdVector ${HEADER_FILES})
find_package(Boost)
catkin_package(
)

if (OpenCV_FOUND)
   include_directories(${OpenCV_INCLUDE_DIRS})
#add_executable(rosbagreadposepluginexe src/main.cpp) 
add_library(rosbagreadposeplugin SHARED src/plugin.cpp)
   target_link_libraries(rosbagreadposeplugin ${catkin_LIBRARIES} ${OpenCV_LIBS} ${rosbag_LIBRARIES} ${std_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} /opt/ros/indigo/lib/librosbag.so /opt/ros/indigo/lib/libimage_view.so /opt/ros/indigo/lib/libimage_transport.so /opt/ros/indigo/lib/libcv_bridge.so ) 
endif()



