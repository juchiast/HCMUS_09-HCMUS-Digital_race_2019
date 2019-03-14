#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include "traffic_sign_detector.hpp"

#include "cds_msgs/SignDetected.h"

SignDetector signDetector;
ros::Publisher signPublisher;

static void visualizeSign(cv::Mat colorImage, const Sign& signObject)
{
    cv::Rect boundingBox{signObject.x, signObject.y, signObject.width, signObject.height};
    cv::Scalar color { 0, 0, 255 };
    cv::rectangle(colorImage, boundingBox, color, 1);

    cv::Point textPoint = {signObject.x, signObject.y};
    std::string text;
    if (signObject.id == 0) {
        text = "Slow";
    } else if (signObject.id == 1) {
        text = "Left";
    } else {
        text = "Right";
    }
    text += ":" + std::to_string(signObject.confident);
    cv::Scalar textColor { 255, 255, 0 };

    int baseLine = 0;
    cv::Size textSize = cv::getTextSize(text, cv::HersheyFonts::FONT_HERSHEY_COMPLEX, 1, 2, &baseLine);
    // cv::rectangle(colorImage, cv::Rect{signObject.x, signObject.y, textSize.width, textSize.height}, cv::Scalar(255, 0, 0), -1);
    cv::putText(colorImage, text, textPoint, cv::HersheyFonts::FONT_HERSHEY_COMPLEX, 1, textColor, 2);
}

static void visualizeSign(cv::Mat colorImage, const std::vector<Sign>& signObjects)
{
    cv::Mat visualizeImage = cv::Mat::zeros(colorImage.size(), CV_8UC3);
    for (const Sign& sign : signObjects)
    {
        visualizeSign(visualizeImage, sign);
    }
    cv::imshow("Signs", visualizeImage);
    cv::waitKey(1);
}

static void convertSignObjectToSignMsg(const Sign& signObject, cds_msgs::SignDetected& signMsg)
{
    signMsg.id = signObject.id;
    signMsg.confident = signObject.confident;
    signMsg.x = signObject.x;
    signMsg.y = signObject.y;
    signMsg.width = signObject.width;
    signMsg.height = signObject.height;
}


static void publishSign()
{
    cds_msgs::SignDetected msg;

    msg.header.stamp = ros::Time::now();

    const std::vector<Sign>&& signs = signDetector.getDetections();

    if (!signs.empty())
    {
        float maxConfident = signs[0].confident;
        size_t maxId = 0;

        for(size_t i = 1; i < signs.size(); i++)
        {
            if (signs[i].confident > maxConfident)
            {
                maxConfident = signs[i].confident;
                maxId = i;
            }
        }

        convertSignObjectToSignMsg(signs[maxId], msg);
        signPublisher.publish(msg);
    }
}

static void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        signDetector.detect(cv_ptr->image);
        visualizeSign(cv_ptr->image, signDetector.getDetections());
        publishSign();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traffic_sign_detector");

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
}