#ifndef TRAFFIC_SIGN_DETECTOR_HPP
#define TRAFFIC_SIGN_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

#include "cds_msgs/SignDetected.h"

class SignRecognizer;

struct Sign
{
    int id;
    cv::Rect boundingBox;
    float confident;
};

class SignDetector
{
public:
    SignDetector(SignRecognizer* recognizer);
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

    std::vector<Sign> signs;
    SignRecognizer* signRecognizer;
};

#endif