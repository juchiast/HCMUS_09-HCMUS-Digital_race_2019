#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "cds_msgs/Lane.h"
#include "cds_msgs/SignDetected.h"
#include <vector>
#include <opencv2/core.hpp>


class Navigation
{
public:

    typedef std::vector<cv::Point> Lane;
    typedef std::vector<bool> TurningFlags;

    Navigation();

    void update(const Lane& leftLane, const Lane& rightLane);
    void update(const TurningFlags& leftTurnFlags, const TurningFlags& rightTurnFlags);
    void update(const cds_msgs::SignDetected& sign);
    float getSpeed();
    float getSteer();

private:
    float errorAngle(const cv::Point &dst);
    float getSteerTurning();
    bool isTurning() const;
    void turnLeft();
    void turnRight();
    void forward();

private:
    Lane leftLane, rightLane;
    TurningFlags leftTurn, rightTurn;
    int sign;

    static const int MIN_VELOCITY = 10;
    static const int MAX_VELOCITY = 30;
    static const int DEF_VELOCITY = 20;
    static const int LANE_WIDTH = 50;


    static const int NONE = -1;
    static const int LEFT = 1;
    static const int RIGHT = 2;

    float currentSpeed;
    float currentSteer;

    static const cv::Point carPos;
    static const cv::Point distanceNearLaneLine;
};


#endif