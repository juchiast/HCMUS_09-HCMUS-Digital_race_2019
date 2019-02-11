#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <team405/Sign.h>

#include "utils.h"
#include "signdetector.h"

constexpr const char* NODE_NAME = "sign_detector";
constexpr const char* TOPIC_SIGN = "team405_sign";
constexpr const char* TOPIC_IMAGE = "team405_image";

static SignDetector* signDetector;
static ros::Publisher signPublisher;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        int key = cv::waitKey(1);
        if (key == 'c' || key == 'C') {
            captureImage(cv_ptr->image);
        }

        signDetector->detect(cv_ptr->image);
        for (const auto& detectedSign : signDetector->getTrafficSign())
        {
            signPublisher.publish(detectedSign);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_IMAGE, 1, imageCallback);

    signPublisher = nh.advertise<team405::Sign>(TOPIC_SIGN, 10);    
    signDetector = new SignDetector();

    ros::spin();
    cv::destroyAllWindows();
    delete signDetector;

    return 0;
}
