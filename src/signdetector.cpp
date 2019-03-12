#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "signdetector.h"

#include <unistd.h>

SignDetector::SignDetector()
{

}

SignDetector::~SignDetector()
{

}

cv::Mat SignDetector::deNoise(cv::Mat inputImage) {
    cv::Mat output;
    cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);
    return output;
}

std::vector<cv::Mat> SignDetector::MSER_Features(cv::Mat binary_img) {
    cv::Mat detection;
    cv::Size size(64, 64);
    std::vector<cv::Mat> detections;

    // cv::Ptr<cv::MSER> ms = cv::MSER::create(5, 64, 500, 0.9, 0.1, 200, 1.01, 0.1, 1);
    cv::Ptr<cv::MSER> ms = cv::MSER::create();
    std::vector<std::vector<cv::Point> > regions;
    std::vector<cv::Rect> mser_bbox;
    ms->detectRegions(binary_img, regions, mser_bbox);

    // For every bounding box in the image
    for (cv::Rect i : mser_bbox) {
        // Ratio filter of detected regions
        double ratio = (static_cast<double>(i.height) /
            static_cast<double>(i.width));

        if (ratio > 0.8 && ratio < 1.2) {
            // Crop bounding boxes to get new images
            detection = binary_img(i);

            // Output the vector of images
            detections.push_back(detection);
            this->boxes.push_back(i);
        }
    }
    return detections;
}

void SignDetector::detect(cv::Mat frame)
{
    CV_Assert(!frame.empty());
    this->boxes.clear();

    // Denoise image with gaussian blur
    cv::Mat img_denoise = deNoise(frame);

    cv::Mat hsvImg;
    cv::cvtColor(frame, hsvImg, cv::COLOR_BGR2HSV);

    cv::Mat binary;
    cv::inRange(hsvImg, cv::Scalar(160/2.0, 150,  100), cv::Scalar(210/2.0, 255, 255), binary);
    cv::imshow("sign binary", binary);

    std::vector<std::vector<cv::Point> > cnts;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if (cnts.empty())
    {
        return;
    }

    bool flag_found = false;

    for (std::vector<cv::Point> cnt : cnts)
    {
        double area = cv::contourArea(cnt, false);
        if (area > 160)
        {
            flag_found = true;
            break;
        }
    }

    if (!flag_found)
    {
        return;
    }

    cv::Mat imCnt = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);
    cv::drawContours(imCnt, cnts, -1, cv::Scalar(0,255,0), 2, 8, hierarchy, 0, cv::Point());
    cv::imshow("sign contours", imCnt);

    // Get the detections using MSER
    std::vector<cv::Mat> imgs_mser = MSER_Features(binary);

    // If there are detection in the frame:
    for (size_t i = 0; i < imgs_mser.size(); i++)
    {
        TrafficSign sign = recognize(imgs_mser[i]);

        Sign signObject;
        signObject.id = static_cast<int>(sign);
        signObject.confident = 1.0f;

        signObject.x = this->boxes[i].x;
        signObject.y = this->boxes[i].y;
        signObject.width = this->boxes[i].width;
        signObject.height = this->boxes[i].height;

        signDetections.push_back(signObject);
    }
}

TrafficSign SignDetector::recognize(cv::Mat boundingBox)
{
    CV_Assert(boundingBox.channels() == 1); // assumse binary image

    cv::Mat lowerLeft = boundingBox(cv::Rect{0, boundingBox.rows / 2, boundingBox.cols / 2, boundingBox.rows / 2});
    cv::Mat lowerRight = boundingBox(cv::Rect{boundingBox.cols / 2, boundingBox.rows / 2, boundingBox.cols / 2, boundingBox.rows / 2});

    int lowerLeftCount = cv::countNonZero(lowerLeft);
    int lowerRightCount = cv::countNonZero(lowerRight);

    float lowerLeftRatio = lowerLeftCount / (lowerLeft.rows * lowerLeft.cols);
    float lowerRightRatio = lowerRightCount / (lowerRight.rows * lowerRight.cols);

    if (lowerLeftRatio > lowerRightRatio)
    {
        return TrafficSign::Left;
    } else if (lowerLeftRatio < lowerRightRatio)
    {
        return TrafficSign::Right;
    } else
    {
        return TrafficSign::Slow;
    }
    
}

std::vector<Sign> SignDetector::getDetections() const
{
    return this->signDetections;
}

void SignDetector::visualize(cv::Mat frame) const
{
    cv::Mat image = cv::Mat::zeros(frame.size(), CV_8UC3);
    char buf[50];
    for (const Sign& sign : this->signDetections)
    {
        memset(buf, 0, sizeof(buf));
        snprintf(buf, 50, "%d : %.2f", sign.id, sign.confident);
        std::string text = buf;

        cv::Point pos {sign.x, sign.y};

        cv::rectangle(image, cv::Rect{sign.x, sign.y, sign.width, sign.height}, cv::Scalar(0, 255, 255), 1);
        cv::putText(image, text, pos, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255,255,0), 1);
    }
    cv::imshow("Sign detection", image);
}