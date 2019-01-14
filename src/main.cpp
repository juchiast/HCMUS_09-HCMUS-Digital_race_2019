#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "std_msgs/String.h"

#include "carcontrol.h"


bool STREAM = true;

VideoCapture capture("video.avi");
CarControl *car;
int skipFrame = 1;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        car->driverCar(cv_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void carCallback(const std_msgs::String& msg)
{
    const char* str = msg.data.c_str();
    if (strcmp(str, "R") == 0){
        car->receiveCommand(CommandId::TurnRight);
    } else if (strcmp(str, "L") == 0) {
        car->receiveCommand(CommandId::TurnLeft);
    } else if (strcmp(str, "C") == 0) {
        car->receiveCommand(CommandId::Continue);
    } else if (strcmp(str, "S") == 0) {
        car->receiveCommand(CommandId::Stop);
    }
}

// void videoProcess()
// {
//     Mat src;
//     while (true)
//     {
//         capture >> src;
//         if (src.empty()) break;
        
//         //imshow("View", src);
//         detect->update(src);
//         waitKey(30);
//     }
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    // cv::namedWindow("Binary");
    // cv::namedWindow("Threshold");
    // cv::namedWindow("Bird View");
    // cv::namedWindow("Lane Detect");

    car = new CarControl();

    if (STREAM) {
        // cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("team405_image", 1, imageCallback);

        ros::Subscriber carSubcriber = nh.subscribe("team405_control", 10, carCallback);

        ros::spin();
    } else {
        // videoProcess();
    }
    cv::destroyAllWindows();
}
