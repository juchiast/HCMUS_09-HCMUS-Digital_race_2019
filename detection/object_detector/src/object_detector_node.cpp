#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <cds_msgs/System.h>
#include <cds_msgs/Object.h>
#include <sensor_msgs/Image.h>

using namespace cv;

static bool isStop = false;
static int depthValue = 150;

static int depthLow = 700;
static int depthHigh = 1500;

static ros::Publisher objectPublisher;

static const int NONE = 0;
static const int LEFT = 1;
static const int RIGHT = 2;

static int prevDirection = NONE;

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

static void detectObjectKinect(cv::Mat depthImage)
{
    cv::Mat depthThreshold;
    inRange(depthImage, Scalar(depthLow), Scalar(depthHigh), depthThreshold);
    int height = 0.4 * depthThreshold.rows;
    depthThreshold(cv::Rect(0, height, depthThreshold.cols, depthThreshold.rows - height)) = Scalar(0);
    // cv::imshow("DepthThreshold", depthThreshold);

    // cv::Mat depthBirdview = birdViewTranform(depthThreshold);
    // cv::imshow("DepthBirdview", depthBirdview);

    // Mat depthBirdviewShow;
    // cvtColor(depthBirdview, depthBirdviewShow, cv::COLOR_GRAY2BGR);

    // cv::Mat visualization;
    // depthImage.copyTo(visualization, depthThreshold);
    cv::imshow("DepthThreshold", depthThreshold);

    Mat leftDepth = depthThreshold(cv::Rect(0, 0, depthThreshold.cols/2, depthThreshold.rows));
    Mat rightDepth = depthThreshold(cv::Rect(depthThreshold.cols/2, 0, depthThreshold.cols/2, depthThreshold.rows));

    

    int leftCount = countNonZero(leftDepth);
    int rightCount = countNonZero(rightDepth);

    const int countThreshold = 50*50;
    cds_msgs::Object objectMsg;
    objectMsg.direction = NONE;
    if (leftCount > countThreshold && rightCount > countThreshold)
    {
        if (leftCount > rightCount)
        {
            // ROS_INFO("Object on the left");
            objectMsg.direction = LEFT;
        } else 
        {
            // ROS_INFO("Object on the right");
            objectMsg.direction = RIGHT;
        }
    } else if (leftCount > countThreshold)
    {
        // ROS_INFO("Object on the left");
        objectMsg.direction = LEFT;

    } else if (rightCount > countThreshold)
    {
        // ROS_INFO("Object on the right");
        objectMsg.direction = RIGHT;
    }

    // if (objectMsg.direction != prevDirection)
    {
        objectPublisher.publish(objectMsg);
        prevDirection = objectMsg.direction;
    }
}

static void processDepthKinect(cv_bridge::CvImagePtr& cv_ptr, const sensor_msgs::ImageConstPtr& msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    if (cv_ptr->image.empty())
    {
        return;
    }

    cv::Mat depthImage = cv_ptr->image;
    // cv::imshow("DepthImage", depthImage);
    detectObjectKinect(depthImage);

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
    // cv::namedWindow("config");
    // cv::createTrackbar("depthValue", "config", &depthValue, 255);

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

    objectPublisher = nh.advertise<cds_msgs::Object>("/object_detected", 1);

    ROS_INFO("object_detector_node started");
    ros::spin();

    return 0;
}