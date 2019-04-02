#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <vector>
#include <opencv2/core.hpp>

struct SignDetected 
{
    int id;
    float confident;
    int x;
    int y;
    int width;
    int height;
};


class Navigation
{
public:

    typedef std::vector<cv::Point> Lane;
    typedef std::vector<bool> TurningFlags;
    
    Navigation();

    void update(const Lane& leftLane, const Lane& rightLane);
    void update(const TurningFlags& leftTurnFlags, const TurningFlags& rightTurnFlags);
    void update(const SignDetected& sign);
    float getSpeed();
    float getSteer();


    cv::Point carDir;
    float theta;


    static int MIN_VELOCITY;
    static int MAX_VELOCITY;
    static int DEF_VELOCITY;

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
    int skipFrameCount;

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