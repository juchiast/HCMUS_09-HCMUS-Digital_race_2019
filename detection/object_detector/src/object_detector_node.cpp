#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <cds_msgs/System.h>
#include <sensor_msgs/Image.h>

static bool isStop = false;
static int depthValue = 150;

typedef void (*ProcessDepthFunc)(cv_bridge::CvImagePtr& cv_ptr, const sensor_msgs::ImageConstPtr& msg);
ProcessDepthFunc processDepthFunc = nullptr;

static void processDepthSimulator(cv_bridge::CvImagePtr& cv_ptr, const sensor_msgs::ImageConstPtr& msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (cv_ptr->image.empty())
    {
        return;
    }

    cv::Mat depthImage;
    cv::cvtColor(cv_ptr->image, depthImage, cv::COLOR_BGR2GRAY);
    // cv::imshow("DepthImage", depthImage);

    cv::Mat depthMask;
    cv::inRange(depthImage, cv::Scalar(0), cv::Scalar(depthValue), depthMask);
    cv::imshow("DepthMask", depthMask);

    cv::waitKey(1);
}

static void processDepthKinect(cv_bridge::CvImagePtr& cv_ptr, const sensor_msgs::ImageConstPtr& msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    if (cv_ptr->image.empty())
    {
        return;
    }

    cv::Mat depthImage = cv_ptr->image;
    cv::imshow("DepthImage", depthImage);


    cv::waitKey(1);
}

static void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (isStop)
    {
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        processDepthFunc(cv_ptr, msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s'.", msg->encoding.c_str());
    }
}

static void systemCallback(const cds_msgs::System& msg)
{
    isStop = msg.isStop.data;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "object_detector");
    cv::namedWindow("config");
    cv::createTrackbar("depthValue", "config", &depthValue, 255);

    ros::NodeHandle nh;
    ros::Subscriber depthSub = nh.subscribe("/camera/depth/image_raw", 10, depthImageCallback);
    ros::Subscriber systemSub = nh.subscribe("/system", 10, systemCallback);


    std::string depthDevice = nh.param("/object_detector/depth_device", std::string("simulator"));
    if (depthDevice == "simulator")
    {
        processDepthFunc = processDepthSimulator;
    } else if (depthDevice == "kinect")
    {
        processDepthFunc = processDepthKinect;
    } else 
    {
        ROS_FATAL("depth_device should be \"simulator\" or \"kinect\"");
        return -1;
    }

    ROS_INFO("object_detector_node started");
    ros::spin();

    return 0;
}