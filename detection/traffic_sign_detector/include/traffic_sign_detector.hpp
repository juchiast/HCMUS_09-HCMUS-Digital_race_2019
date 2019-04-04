#ifndef TRAFFIC_SIGN_DETECTOR_HPP
#define TRAFFIC_SIGN_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

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
    SignDetector(std::string left_image_path, std::string right_image_path);
    ~SignDetector();

    void detect(cv::Mat frame);
    const Sign* getSign() const;

private:
    cv::Mat deNoise(cv::Mat inputImage);

private:
    cv::Mat LEFT_TEMPLATE, RIGHT_TEMPLATE;
    double MAX_DIFF;

private:
    Sign signTypeDetected;
};

#endif
