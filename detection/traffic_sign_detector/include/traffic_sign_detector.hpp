#ifndef TRAFFIC_SIGN_DETECTOR_HPP
#define TRAFFIC_SIGN_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

#include <ros/ros.h>
#include "cds_msgs/SignDetected.h"

class SignRecognizer;

enum TrafficSign
{
    None = -1,      // Not a sign
    Left = 1,       // Left sign (should have confident >= SIGN_THRESHOLD)
    Right = 2       // Right sign (should have confident >= SIGN_THRESHOLD)
};

struct Sign
{
    int id;
    cv::Rect boundingBox;
    float confident;
};

class SignDetector
{
public:
    SignDetector();
    ~SignDetector();

    void toSignMessage(cds_msgs::SignDetected& msg);
    void detect(cv::Mat frame);
private:
    cv::Mat deNoise(cv::Mat inputImage);
    const Sign* getSign() const;

private:
    cv::Mat LEFT_TEMPLATE, RIGHT_TEMPLATE;
    double MAX_DIFF;

private:
    Sign signTypeDetected;

    ros::NodeHandle nodeHandle;
};

#endif
