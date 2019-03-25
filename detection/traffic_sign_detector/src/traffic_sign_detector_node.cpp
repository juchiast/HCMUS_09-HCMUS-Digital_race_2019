#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include "traffic_sign_detector.hpp"

#include "cds_msgs/SignDetected.h"

#define VISUALIZE_WIN_NAME "Sign"

// SignRecognizer *signRecognizer = nullptr;
SignDetector *signDetector = nullptr;
ros::Publisher signPublisher;

TrafficSign prevPublishedSign = TrafficSign::None;

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
    cv::RNG rng(123456);
    const cv::Scalar backgroundColor(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    const cv::Scalar foregroundColor(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    const cv::Scalar textColor(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

    cv::Size textSize = cv::getTextSize(signLabel, fontface, scale, thickness, &baseline);
    // cv::Mat crop = colorImage(cv::Rect{signMsg.x, signMsg.y, signMsg.width, signMsg.height});
    // cv::Mat visualizeImage = cv::Mat(cv::Size(crop.cols, crop.rows + textSize.height), CV_8UC3);
    cv::Mat visualizeImage = colorImage.clone();

    cv::Point textPos{signMsg.x, signMsg.y - textSize.height};
    cv::rectangle(visualizeImage, textPos, textPos + cv::Point(textSize.width + 2 * padding, textSize.height + 2 * padding), backgroundColor, CV_FILLED);
    cv::rectangle(visualizeImage, textPos, textPos + cv::Point(textSize.width + 2 * padding, textSize.height + 2 * padding), foregroundColor, 1, cv::LineTypes::LINE_4);
    cv::putText(visualizeImage, signLabel, textPos + cv::Point(padding, padding + baseline * 2), fontface, scale, textColor, thickness, 8);

    cv::imshow(VISUALIZE_WIN_NAME, visualizeImage);
    cv::waitKey(1);
}

static void publishSign(cv::Mat image)
{
    signDetector->detect(image);

    cds_msgs::SignDetected msg;
    msg.header.stamp = ros::Time::now();

    signDetector->toSignMessage(msg);
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

    // signRecognizer = new BinarySignRecognizer();
    signDetector = new SignDetector();

    ros::NodeHandle nh;

    std::string camera_topic;
    nh.param<std::string>("sub_camera_topic", camera_topic, "/camera/rgb/image_raw");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(camera_topic, 1, imageCallback);

    std::string pub_sign_topic;
    nh.param<std::string>("pub_sign_topic", pub_sign_topic, "/sign_detected");
    signPublisher = nh.advertise<cds_msgs::SignDetected>(pub_sign_topic, 10);

    ros::spin();

    cv::destroyAllWindows();
    // delete signRecognizer;
    delete signDetector;
}
