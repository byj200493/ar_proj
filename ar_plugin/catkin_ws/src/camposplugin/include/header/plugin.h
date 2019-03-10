#include<ros/ros.h>
#include <rosbag/bag.h>
       #include <rosbag/view.h>
       #include <std_msgs/Int32.h>
       #include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>//
#include <iostream>   
#include <fstream>
       #include <boost/foreach.hpp>
       #define foreach BOOST_FOREACH

extern "C"{
void initplugin(const char *thepath);
int getpose(float *poses);
void getNewImage(int *imgBuf);
void getWalls(float *quadBuf, int *numVertices);
void getBoxes(float *boxBuf, int *numVertices);
int getwidth();
int getheight();
}
