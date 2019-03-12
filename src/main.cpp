#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "carcontrol.h"


CarControl* car = nullptr;

static void colorImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        car->drive(cv_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

static void distanceSensorCallback(const std_msgs::Bool& msg)
{
    if (msg.data == false) {
        car->stop();
    }
}

static std::string getTopicValue(ros::NodeHandle& nodeHandle, const std::string& name, const std::string& defaultVal)
{
    std::string res = defaultVal;
    if (!nodeHandle.getParam(name, res))
    {
        ROS_INFO("Not found param [%s], use default = [%s]", name.c_str(), defaultVal.c_str());
    }
    return res;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "team405");

    car = new CarControl();

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    std::string topicColorCamera = getTopicValue(nh, "camera_rgb", "/camera/rgb/image_raw");
    image_transport::Subscriber sub = it.subscribe(topicColorCamera, 1, colorImageCallback);


    ros::spin();

    car->stop();
    delete car;
    cv::destroyAllWindows();
}
