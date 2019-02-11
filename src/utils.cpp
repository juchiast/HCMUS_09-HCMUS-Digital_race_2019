#include "utils.h"
#include <string>
#include <ros/ros.h>
#include <opencv2/highgui.hpp>

void captureImage(const cv::Mat& image)
{
    const std::string time = std::to_string(ros::Time::now().toNSec());
    const std::string name = "./capture_" + time + ".jpg";
    bool flag = cv::imwrite(name, image);
    if (flag) {
        ROS_INFO("Capture: %s", name.c_str());
    } else {
        ROS_INFO("Cannot capture image");
    }
}