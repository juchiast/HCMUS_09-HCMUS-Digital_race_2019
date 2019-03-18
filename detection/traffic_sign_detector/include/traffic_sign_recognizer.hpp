#ifndef TRAFFIC_SIGN_RECOGNIZER_HPP
#define TRAFFIC_SIGN_RECOGNIZER_HPP

#include <opencv2/core.hpp>

enum TrafficSign
{
    None = -1,      // Not a sign
    Slow = 0,       // May be a sign (confident < SIGN_THRESHOLD)
    Left = 1,       // Left sign (should have confident >= SIGN_THRESHOLD)
    Right = 2       // Right sign (should have confident >= SIGN_THRESHOLD)
};

class SignRecognizer
{
public:
    virtual void recognize(cv::Mat sign, TrafficSign& signIdResult, double& confident) = 0;
    
private:

};

class BinarySignRecognizer : public SignRecognizer
{
public:
    cv::Mat toBinary(cv::Mat image);
    virtual void recognize(cv::Mat sign, TrafficSign& signIdResult, double& confident);
};

// Maybe class ColorSignRecognizer to recognize sign from color image?


#endif