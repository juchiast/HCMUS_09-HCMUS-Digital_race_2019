#include <ros/ros.h>

#include "cds_msgs/SignDetected.h"
#include "cds_msgs/Lane.h"

#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>

cds_msgs::SignDetected lastSignMsg;
// cv::VideoWriter recorder;

int skyLine = 85;
int BIRDVIEW_WIDTH = 240;
int BIRDVIEW_HEIGHT = 320;
int delta_width_bot = 105;

int low_H = 0, low_S = 0, low_V = 180, high_H = 179, high_S = 65, high_V = 255;

static void signMsgCallback(const cds_msgs::SignDetected &msg)
{
    lastSignMsg = msg;
}


cv::Mat birdViewTranform(const cv::Mat &src)
{
    using namespace cv;
    Point2f src_vertices[4];

    int width = src.size().width;
    int height = src.size().height;

    src_vertices[0] = Point(0, skyLine);
    src_vertices[1] = Point(width, skyLine);
    src_vertices[2] = Point(width, height);
    src_vertices[3] = Point(0, height);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(BIRDVIEW_WIDTH, 0);
    dst_vertices[2] = Point(BIRDVIEW_WIDTH - delta_width_bot, BIRDVIEW_HEIGHT);
    dst_vertices[3] = Point(delta_width_bot, BIRDVIEW_HEIGHT);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);

    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    return dst;
}


static void processImageCallback(cv::Mat frame)
{

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat binary;
    cv::inRange(hsv, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), binary);
    cv::imshow("Binary_config", binary);

    // cv::Mat binaryColor;
    // cv::cvtColor(binary, binaryColor, cv::COLOR_GRAY2BGR);

    // cv::Mat visualize;
    // cv::hconcat(frame, binaryColor, visualize);

    // cv::imshow("LineSegement", visualize);

    // binary = birdViewTranform(binary);
    // binaryColor = birdViewTranform(binaryColor);

    // cv::Mat frameIPM = birdViewTranform(frame);

    // cv::hconcat(frameIPM, binaryColor, visualize);
    // cv::imshow("birdview", visualize);
    cv::waitKey(1);
}

static void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // recorder.write(cv_ptr->image);
        // cv::imshow("recording...", cv_ptr->image);
        processImageCallback(cv_ptr->image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualization");
    ros::NodeHandle nh;


    cv::namedWindow("config");
    

    cv::createTrackbar("low hue", "config", &low_H, 180);
    cv::createTrackbar("high hue", "config", &high_H, 180);
    cv::createTrackbar("low saturation", "config", &low_S, 255);
    cv::createTrackbar("high saturation", "config", &high_S, 255);
    cv::createTrackbar("low value", "config", &low_V, 255);
    cv::createTrackbar("high value", "config", &high_V, 255);

    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 10, imageCallback);

    // recorder = cv::VideoWriter("outcpp.avi",CV_FOURCC('M','J','P','G'),10, cv::Size(320, 240));

    ROS_INFO("Start recording..");
    ros::spin();

    // recorder.release();

    return 0;
}