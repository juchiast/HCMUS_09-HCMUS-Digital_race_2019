#include "road_segmentation.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

void RoadSegmentation::detect(cv::Mat frame)
{
    // for simple now, just use color threshold

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(), cv::Scalar(180, 32, 100), binary);
}

cv::Mat RoadSegmentation::getSegmentation() const
{
    return this->binary;
}