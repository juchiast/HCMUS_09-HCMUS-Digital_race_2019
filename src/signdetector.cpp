#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "ros/console.h"
#include "signdetector.h"

#include <unistd.h>
#include <iostream>

SignDetector::SignDetector()
{
    char cwd[255];
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        ROS_INFO("Current working dir: %s\n", cwd);
    }
}

SignDetector::~SignDetector()
{

}

cv::Mat SignDetector::deNoise(cv::Mat inputImage) {
    cv::Mat output;
    cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);
    return output;
}

std::vector<cv::Rect> SignDetector::MSER_Features(cv::Mat binary_img) {
    cv::Mat detection;
    cv::Size size(64, 64);
    std::vector<cv::Mat> detections;

    // cv::Ptr<cv::MSER> ms = cv::MSER::create(5, 64, 500, 0.9, 0.1, 200, 1.01, 0.1, 1);
    cv::Ptr<cv::MSER> ms = cv::MSER::create();
    std::vector<std::vector<cv::Point> > regions;
    std::vector<cv::Rect> mser_bbox;
    ms->detectRegions(binary_img, regions, mser_bbox);


    // For every bounding box in the image

    for (auto iter = mser_bbox.begin(); iter != mser_bbox.end();)
    {
        cv::Rect i = *iter;
        // Ratio filter of detected regions
        double ratio = (static_cast<double>(i.height) / static_cast<double>(i.width));

        if (!(ratio > 0.8 && ratio < 1.2))
        {
            iter = mser_bbox.erase(iter);
        }
        else
        {
            iter++;
        }
    }
    return mser_bbox;
}

void SignDetector::detect(cv::Mat frame)
{
    CV_Assert(!frame.empty());

    // Denoise image with gaussian blur
    cv::Mat img_denoise = deNoise(frame);

    cv::Mat hsvImg;
    cv::cvtColor(frame, hsvImg, cv::COLOR_BGR2HSV);

    cv::Mat binary;
    cv::inRange(hsvImg, cv::Scalar(160/2.0, 150,  100), cv::Scalar(210/2.0, 255, 255), binary);
    cv::imshow("binary", binary);

    std::vector<std::vector<cv::Point> > cnts;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (auto iter = cnts.begin(); iter != cnts.end();)
    {
        auto cnt = *iter;
        double area = cv::contourArea(cnt);
        if (area <= 40)
        {
            iter = cnts.erase(iter);
        } else
        {
            iter++;
        }
    }

    // cv::Mat imCnt = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);
    // cv::drawContours(imCnt, cnts, -1, cv::Scalar(0,255,0), 2, 8, hierarchy, 0, cv::Point());
    // cv::imshow("contours", imCnt);
    std::vector<cv::Rect> imgs_mser = MSER_Features(binary);
    this->signs.clear();
    // If there are detection in the frame:
    for (auto rect : imgs_mser)
    {
        cv::Mat cropped = binary(rect);
        cv::Mat lower_left = cropped(cv::Rect(0, cropped.rows / 2, cropped.cols / 2, cropped.rows / 2));
        cv::Mat lower_right = cropped(cv::Rect(cropped.cols / 2, cropped.rows / 2, cropped.cols / 2, cropped.rows / 2));

        int blue_component_lower_left = cv::countNonZero(lower_left);
        int blue_component_lower_right = cv::countNonZero(lower_right);
        
        team405::Sign sign;
        sign.row = rect.y;
        sign.col = rect.x;
        sign.width = rect.width;
        sign.height = rect.height;
        
        if (blue_component_lower_left > blue_component_lower_right)
        {
            sign.direction = TrafficSign::Left;
        } else if (blue_component_lower_left < blue_component_lower_right)
        {
            sign.direction = TrafficSign::Right;
        } else 
        {
            ROS_INFO("Cannot detect!!");
        }

        this->signs.push_back(sign);
    }
}

std::vector<team405::Sign> SignDetector::getTrafficSign()
{
    return this->signs;
}