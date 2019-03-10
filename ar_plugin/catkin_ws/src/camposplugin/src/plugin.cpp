#include </home/dev/Code/ar_proj/ar_plugin/catkin_ws/src/camposplugin/include/header/plugin.h>
#include <stdlib.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
int argc=1;
const int width =  224*2.7;
const int height = 376;
unsigned char *rgb_bytes;
//std::vector<Eigen::Vector3f> _quadVertices;
//std::vector<Eigen::Vector3f> _boxVertices;
typedef struct Vector3{
    float x,y,z;
}Vector3;
std::vector<Vector3> quadVertices;
std::vector<Vector3> boxVertices;
image_transport::Subscriber color_sub;

ros::Subscriber sub_box_points;
ros::Subscriber sub_quad_points;
ros::Subscriber campos_sub;
ros::Subscriber unityCamPose_sub;
cv::Mat _rgbMat;
bool _bColorCaptured = false;
bool  gotsomething = false;
float *poses;
float *_quadBuf;
float *_boxBuf;
int quad_vt_count = 0;
int box_vt_count = 0;

void cpyImageMsg(const sensor_msgs::ImageConstPtr& msg,
                 sensor_msgs::ImagePtr &img,
                 cv::Mat &cv_image)
{
    boost::shared_ptr<void const> tracked_object;
    try
    {
      cv_image = cv_bridge::toCvShare(*msg, tracked_object, msg->encoding)->image.clone();
    }
    catch (cv::Exception &e)
    {
      ROS_ERROR("Could not convert from '%s' to '%s'.", msg->encoding.c_str(), msg->encoding.c_str());
      return;
    }
    img = cv_bridge::CvImage(msg->header, msg->encoding, cv_image).toImageMsg();/*"bgr8"*/
}
//grab rgb image from callback
void getRGBImageFromMsg(cv::Mat &cv_image){
    int imgfillcount=0;
    for(int i = 0;i<width;i++)
    {
        for(int j = height-1; j >= 0; --j)
        {
            int b = cv_image.at<cv::Vec3b>(j,i)[0];
            int g = cv_image.at<cv::Vec3b>(j,i)[1];
            int r = cv_image.at<cv::Vec3b>(j,i)[2];
            rgb_bytes[imgfillcount]=(unsigned char)r;
            imgfillcount++;
            rgb_bytes[imgfillcount]=(unsigned char)g;
            imgfillcount++;
            rgb_bytes[imgfillcount]=(unsigned char)b;
            imgfillcount++;
        }
    }
    std::cout<<"get rgb image from msg done\n";
}
//callback functions
void color_imageCallback(const sensor_msgs::ImageConstPtr& msgRGB)
{
    _bColorCaptured = true;
    sensor_msgs::ImagePtr rgb_msg;
    cpyImageMsg(msgRGB, rgb_msg, _rgbMat);
    getRGBImageFromMsg(_rgbMat);
}

void boxCallback(const visualization_msgs::Marker::ConstPtr& markerMsg)
{
    box_vt_count = markerMsg->points.size();
    for(int i=0; i < markerMsg->points.size(); ++i)
    {
        _boxBuf[3*i] = markerMsg->points[i].x;
        _boxBuf[3*i+1] = markerMsg->points[i].y;
        _boxBuf[3*i+2] = markerMsg->points[i].z;
    }
}

void quadCallback(const visualization_msgs::Marker::ConstPtr& markerMsg)
{
    quad_vt_count = markerMsg->points.size();
    for(int i=0; i < markerMsg->points.size(); ++i)
    {
        _quadBuf[3*i] = (float)markerMsg->points[i].x;
        _quadBuf[3*i+1] = (float)markerMsg->points[i].y;
        _quadBuf[3*i+2] = (float)markerMsg->points[i].z;
    }
}

//void camposCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
void camposCallback(const nav_msgs::OdometryConstPtr& msg)
{
    poses[0] = msg->pose.pose.position.x;
    poses[1] = msg->pose.pose.position.y;
    poses[2] = msg->pose.pose.position.z;
    poses[3] = msg->pose.pose.orientation.w;
    poses[4] = msg->pose.pose.orientation.x;
    poses[5] = msg->pose.pose.orientation.y;
    poses[6] = msg->pose.pose.orientation.z;
    gotsomething = true;
}

void unityCamPoseCallback(const visualization_msgs::Marker::ConstPtr& poseMsg)
{
    poses[0] = poseMsg->points[0].x;
    poses[1] = poseMsg->points[0].y;
    poses[2] = poseMsg->points[0].z;
    poses[3] = poseMsg->points[1].x;
    poses[4] = poseMsg->points[1].y;
    poses[5] = poseMsg->points[1].z;
    poses[6] = poseMsg->points[2].x;
    poses[7] = poseMsg->points[2].y;
    poses[8] = poseMsg->points[2].z;
    gotsomething = true;
}

//plugin functions (unity has access to)
void initplugin(const char *thepath)
{
    rgb_bytes=new unsigned char[width*height*3];
    _quadBuf = new float[1000];
    _boxBuf = new float[3000];
    poses = new float[9];
    poses[0] = 1;
    poses[1] = 1;
    poses[2] = 1;
    poses[3] = 1;
    poses[4] = 0;
    poses[5] = 0;
    poses[6] = 0;
    poses[7] = 0;
    poses[8] = 0;
    int charcount=0;
    for(int i=0;thepath[i]!='\0';i++)
    {
      charcount++;
    }
    char *argv=new char[charcount+1];
    for(int i=0;i<charcount+1;i++)
    {
      argv[i]=thepath[i];
    }

    ros::init(argc, &argv, "rosbagunityreadposeplugin");
    std::cout<<"ros imitialized\n";
    ros::NodeHandle sub_nh;
    image_transport::ImageTransport sub_it(sub_nh);
    color_sub = sub_it.subscribe("camera/image", 15, color_imageCallback);// "/asus/rgb/image_raw"
    sub_box_points = sub_nh.subscribe("BoundingBox/points", 15, boxCallback);
    sub_quad_points = sub_nh.subscribe("WallQuad/points", 15, quadCallback);
    //campos_sub = sub_nh.subscribe("camera/pose", 1,camposCallback);//"/asus/odometry"
    unityCamPose_sub = sub_nh.subscribe("camera/unityPose", 15, unityCamPoseCallback);
}

int getpose(float *pose)
{
    //position
     pose[0] = poses[0];
     pose[1] = poses[1];
     pose[2] = poses[2];
     //forward
     pose[3] = poses[3];
     pose[4] = poses[4];
     pose[5] = poses[5];
     //upward
     pose[6] = poses[6];
     pose[7] = poses[7];
     pose[8] = poses[8];
     ros::spinOnce();
     if(gotsomething==false)
         return 0;
     gotsomething = false;
     return 1;
}

void getNewImage(int *imgBuf)
{
    for(int i=0;i<width*height*3;i++)
    {
       imgBuf[i]=(int)rgb_bytes[i];
    }
}

void getWalls(float *quadBuf, int *numVertices)
{
    numVertices[0] = quad_vt_count;
    for(int i=0; i < 3*quad_vt_count; ++i)
    {
        quadBuf[i] = _quadBuf[i];
    }
}

void getBoxes(float *boxBuf, int *numVertices)
{
    numVertices[1] = box_vt_count;
    for(int i=0; i < 3*box_vt_count; ++i)
    {
        boxBuf[i] = _boxBuf[i];
    }
}

int getwidth()
{
    return width;
}

int getheight()
{
    return height;
}
