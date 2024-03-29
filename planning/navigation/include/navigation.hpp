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

    void update(Lane& leftLane, Lane& rightLane);
    void update(const TurningFlags& leftTurnFlags, const TurningFlags& rightTurnFlags);
    void update(const SignDetected& sign);
    void updateObjectDirection(const int& dir);
    float getSpeed();
    float getSteer();

    void visualize();
    
    Lane approximateLeftLane(const Lane& rightLane);
    Lane approximateRightLane(const Lane& leftLane);


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
    int objectDir;

    static const int LANE_WIDTH = 50;


    static const int NONE = -1;
    static const int LEFT = 1;
    static const int RIGHT = 2;

    float currentSpeed;
    float currentSteer;

    cv::Mat visualizeImage;
    static const cv::Point carPos;
    static const cv::Point distanceNearLaneLine;
};


#endif