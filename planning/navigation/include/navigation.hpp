#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "cds_msgs/Lane.h"
#include "cds_msgs/SignDetected.h"
#include <vector>
#include <opencv2/core.hpp>

typedef std::vector<cv::Point> Lane;

class Navigation
{
public:
    Navigation();

    void update(const Lane& leftLane, const Lane& rightLane);
    void update(const cds_msgs::SignDetected& sign);
    float getSpeed();
    float getSteer();

private:
    float errorAngle(const cv::Point &dst);
    float getSteerTurning();
    void turnLeft();
    void turnRight();
    void forward();

private:
    Lane leftLane, rightLane;
    int sign;
    bool isTurning;

    static const int MIN_VELOCITY = 10;
    static const int MAX_VELOCITY = 30;
    static const int LANE_WIDTH = 50;


    static const int NONE = -1;
    static const int LEFT = 1;
    static const int RIGHT = 2;

    float currentSpeed;
    float currentSteer;

    cv::Point carPos;
};


#endif