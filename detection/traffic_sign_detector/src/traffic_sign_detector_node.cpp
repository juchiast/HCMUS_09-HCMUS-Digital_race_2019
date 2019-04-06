#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include "traffic_sign_detector.hpp"

#include "cds_msgs/SignDetected.h"

#define VISUALIZE_WIN_NAME "Sign"

SignDetector *signDetector = nullptr;
ros::Publisher signPublisher;
cv::RNG rng(123456);

TrafficSign prevPublishedSign = TrafficSign::None;


static void toSignMessage(cds_msgs::SignDetected &msg)
{
  const Sign *sign = signDetector->getSign();
  if (sign == nullptr)
  {
    msg.id = TrafficSign::None;
  }
  else
  {
    msg.id = static_cast<int>(sign->id);
    msg.x = sign->boundingBox.x;
    msg.y = sign->boundingBox.y;
    msg.width = sign->boundingBox.width;
    msg.height = sign->boundingBox.height;
  }
}

static std::string getSignLabel(const cds_msgs::SignDetected &signMsg)
{
    switch (signMsg.id)
    {
    case TrafficSign::Left:
        return "left";
    case TrafficSign::Right:
        return "right";
    }
    return "";
}

static void visualizeSign(cv::Mat colorImage, const cds_msgs::SignDetected &signMsg)
{
    std::string signLabel = getSignLabel(signMsg);
    int fontface = cv::FONT_HERSHEY_DUPLEX;
    double scale = 0.5;
    int thickness = 1;
    int baseline = 0;
    int padding = 2;
    const cv::Scalar backgroundColor(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    const cv::Scalar foregroundColor(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    const cv::Scalar textColor(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

    cv::Size textSize = cv::getTextSize(signLabel, fontface, scale, thickness, &baseline);
    cv::Mat visualizeImage = colorImage.clone();

    cv::Point textPos{signMsg.x, signMsg.y - textSize.height};
    cv::rectangle(visualizeImage, textPos, textPos + cv::Point(textSize.width + 2 * padding, textSize.height + 2 * padding), backgroundColor, CV_FILLED);
    cv::rectangle(visualizeImage, textPos, textPos + cv::Point(textSize.width + 2 * padding, textSize.height + 2 * padding), foregroundColor, 1, cv::LineTypes::LINE_4);
    cv::putText(visualizeImage, signLabel, textPos + cv::Point(padding, padding + baseline * 2), fontface, scale, textColor, thickness, 8);

    ROS_INFO("ASDSKLAJDKASLJDK");
    cv::imshow(VISUALIZE_WIN_NAME, visualizeImage);
    cv::waitKey(1);
}

static void publishSign(cv::Mat image)
{
    signDetector->detect(image);

    cds_msgs::SignDetected msg;
    msg.header.stamp = ros::Time::now();

    toSignMessage(msg);
    visualizeSign(image, msg);

    if (!(msg.id == TrafficSign::None && prevPublishedSign == TrafficSign::None))
    {        
        signPublisher.publish(msg);
        prevPublishedSign = static_cast<TrafficSign>(msg.id);
    }
}

static void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        ROS_INFO("image callback!!!");
        publishSign(cv_ptr->image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traffic_sign_detector");

    ros::NodeHandle nh;
    std::string left_image_path = nh.param("/traffic_sign_detector/left_image_path", std::string("/home/nvidia/left.png"));
    std::string right_image_path = nh.param("/traffic_sign_detector/right_image_path", std::string("/home/nvidia/right.png"));

    ROS_INFO("Left image path = %s", left_image_path.c_str());
    ROS_INFO("Right image path = %s", right_image_path.c_str());


    signDetector = new SignDetector(left_image_path, right_image_path);

    ros::Subscriber sub = nh.subscribe("/camera/rgb/image_raw", 1, imageCallback);

    signPublisher = nh.advertise<cds_msgs::SignDetected>("sign_detected", 10);

    ROS_INFO("sign_detector node started");

    ros::spin();

    cv::destroyAllWindows();

    delete signDetector;
}
