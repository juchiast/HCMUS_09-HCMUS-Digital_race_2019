#include "traffic_sign_recognizer.hpp"
#include <ros/ros.h>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

cv::Mat BinarySignRecognizer::toBinary(cv::Mat sign)
{
    cv::Mat hsvImg;
    cv::cvtColor(sign, hsvImg, cv::COLOR_BGR2HSV);
    
    cv::Mat binary;
    cv::inRange(hsvImg, cv::Scalar(160/2.0, 150,  100), cv::Scalar(210/2.0, 255, 255), binary);

    return binary;
}

void BinarySignRecognizer::recognize(cv::Mat sign, TrafficSign& signIdResult, double& confident)
{
    cv::Mat binarySign = sign;
    if (binarySign.channels() != 1)
    {
        std::cerr << "Not a binary image" << std::endl;
        binarySign = toBinary(binarySign);
    }

    cv::Mat lowerLeft = binarySign(cv::Rect{0, binarySign.rows / 2, binarySign.cols / 2, binarySign.rows / 2});
    cv::Mat lowerRight = binarySign(cv::Rect{binarySign.cols / 2, binarySign.rows / 2, binarySign.cols / 2, binarySign.rows / 2});

    int lowerLeftCount = cv::countNonZero(lowerLeft);
    int lowerRightCount = cv::countNonZero(lowerRight);
    float ratio = lowerLeftCount * 1.0f / lowerRightCount;

    if (ratio > 1.1f)
    {
        signIdResult = TrafficSign::Left;
    } else if (ratio < 0.9f)
    {
        signIdResult = TrafficSign::Right;
    } else
    {
        signIdResult = TrafficSign::Slow;
    }
    confident = 1;

    // {
    //     cv::imshow("Sign " + std::to_string(signIdResult), sign);

    //     cv::Mat lowerLeft = sign(cv::Rect{0, sign.rows / 2, sign.cols / 2, sign.rows / 2});
    //     cv::Mat lowerRight = sign(cv::Rect{sign.cols / 2, sign.rows / 2, sign.cols / 2, sign.rows / 2});
    //     cv::Mat debugSignImage;
    //     cv::hconcat(lowerLeft, lowerRight, debugSignImage);
    //     cv::imshow("Left Right" + std::to_string(signIdResult), debugSignImage);
    //     ROS_INFO("Ratio %.2f", ratio);
    // }

}