#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "cds_msgs/Lane.h"
#include "cds_msgs/SignDetectedArray.h"
#include <vector>
#include <opencv2/core.hpp>

typedef std::vector<cv::Point>& Lane;

class Navigation
{
public:
    Navigation();

    void perceptionCallback(cds_msgs::LaneConstPtr laneMsg, cds_msgs::SignDetectedArrayConstPtr signMsg);
    void update(const Lane& leftLane, const Lane& rightLane, const std::vector<cds_msgs::SignDetected>& signs);
    float getSpeed() const;
    float getSteer() const;

private:
    float errorAngle(const cv::Point &dst);
    void convertLandMarkMsg2Lane(const std::vector<cds_msgs::LandMark>& landmarkMsg, Lane& lane);

private:
    float currentSpeed;
    float currentSteer;

    cv::Point carPos;
};


#endif