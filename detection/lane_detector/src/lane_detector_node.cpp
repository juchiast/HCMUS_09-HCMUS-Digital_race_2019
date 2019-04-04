#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include "lane_detector.hpp"

#include "cds_msgs/Lane.h"
#include "std_msgs/String.h"
#include "cds_msgs/System.h"

LaneDetector laneDetector;
ros::Publisher lanePublisher;

static bool isStop = true;

static void publishLane()
{
    cds_msgs::Lane msg;
    std::vector<cv::Point> leftLane = laneDetector.getLeftLane();
    std::vector<cv::Point> rightLane = laneDetector.getRightLane();
    std::vector<bool> leftTurn = laneDetector.getLeftTurn();
    std::vector<bool> rightTurn = laneDetector.getRightTurn();

    for (const cv::Point& point : leftLane)
    {
        cds_msgs::LandMark landMarkMsg;
        landMarkMsg.x = point.x;
        landMarkMsg.y = point.y;
        msg.leftLane.push_back(landMarkMsg);
    }

    for (const cv::Point& point : rightLane)
    {
        cds_msgs::LandMark landMarkMsg;
        landMarkMsg.x = point.x;
        landMarkMsg.y = point.y;
        msg.rightLane.push_back(landMarkMsg);
    }

    for (auto val : leftTurn)
    {
        std_msgs::Bool valMsg;
        valMsg.data = val;
        msg.leftTurn.push_back(valMsg);
    }

    for (auto val : rightTurn)
    {
        std_msgs::Bool valMsg;
        valMsg.data = val;
        msg.rightTurn.push_back(valMsg);
    }

    lanePublisher.publish(msg);
}

static void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (isStop)
    {
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        laneDetector.updateColorImage(cv_ptr->image);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
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
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        laneDetector.updateDepthImage(cv_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

static void systemCallback(const cds_msgs::System& msg){
    isStop = msg.isStop.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detector");

    ros::NodeHandle nh;


    ros::Subscriber sub = nh.subscribe("/camera/rgb/image_raw", 1, imageCallback);
    ros::Subscriber subdepth = nh.subscribe("/camera/depth/image_raw", 1, depthImageCallback);



    ros::Subscriber systemSub = nh.subscribe("/system", 1, systemCallback);

    if (!nh.getParam("/lane_detector/birdview_width", LaneDetector::BIRDVIEW_WIDTH))
    {
        ROS_WARN("Not found param birdview_width, use default 240");
        LaneDetector::BIRDVIEW_WIDTH = 240;
    }

    if (!nh.getParam("/lane_detector/birdview_height", LaneDetector::BIRDVIEW_HEIGHT))
    {
        ROS_FATAL("Not found param birdview_height, use default 320");
        LaneDetector::BIRDVIEW_HEIGHT = 320;
    }

    if (!nh.getParam("/lane_detector/skyline", LaneDetector::SKYLINE))
    {
        ROS_FATAL("Not found param skyline, use default 85");
        LaneDetector::SKYLINE = 85;
    }

    if (!nh.getParam("/lane_detector/birdview_bottom_delta", LaneDetector::BIRDVIEW_BOTTOM_DELTA))
    {
        ROS_FATAL("Not found param birdview_bottom_delta, use default 105");
        LaneDetector::BIRDVIEW_BOTTOM_DELTA = 105;
    }

    lanePublisher = nh.advertise<cds_msgs::Lane>("lane_detected", 10);

    ROS_INFO("lane_detector node started");


    ros::Rate rate{30};
    while (ros::ok())
    {
        ros::spinOnce();
        if (!isStop)
        {
            laneDetector.detect();
            publishLane();
        }
        rate.sleep();
    }

    cv::destroyAllWindows();
}
