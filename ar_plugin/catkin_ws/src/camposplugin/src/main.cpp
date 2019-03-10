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
       #include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <geometry_msgs/PoseStamped.h>
//Subscriber campos_sub;
bool  gotsomething = false;
void camposCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
std::cout<<"got some pos\n";
	std::cout<< msg->pose.position.x << std::endl;
  
}

//plugin functions (unity has access to)
int main(int argc, char**argv)
{
 std::cout<<"program started\n";
 ros::init(argc, argv, "rosbagunityreadplugin");
std::cout<<"ros imitialized\n";

//setup callback functions      
ros::NodeHandle sub_nh;
//geometry_msgs::PoseStamped sub_it(sub_nh);

ros::Subscriber campos_sub = sub_nh.subscribe("/vins_estimator/camera_pose", 1,camposCallback);
std::cout<<"plugin imitialized\n";
while(true){
ros::spinOnce();
}
}





