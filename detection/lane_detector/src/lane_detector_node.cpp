#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include "lane_detector.hpp"

#include "cds_msgs/Lane.h"
#include "std_msgs/String.h"

LaneDetector laneDetector;
ros::Publisher lanePublisher;

static void publishLane()
{
    cds_msgs::Lane msg;
    std::vector<cv::Point> leftLane = laneDetector.getLeftLane();
    std::vector<cv::Point> rightLane = laneDetector.getRightLane();

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

    lanePublisher.publish(msg);
}

static void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        laneDetector.detect(cv_ptr->image);
        publishLane();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detector");

    ros::NodeHandle nh;

    std::string camera_topic;
    nh.param<std::string>("sub_camera_topic", camera_topic, "/camera/rgb/image_raw");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(camera_topic, 1, imageCallback);

    std::string pub_lane_topic;
    nh.param<std::string>("pub_lane_topic", pub_lane_topic, "/lane_detected");
    lanePublisher = nh.advertise<cds_msgs::Lane>(pub_lane_topic, 10);

    ros::spin();

    cv::destroyAllWindows();
}
