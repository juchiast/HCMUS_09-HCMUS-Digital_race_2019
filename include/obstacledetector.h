#ifndef __OBSTACLE_DETECTOR_H
#define __OBSTACLE_DETECTOR_H

#include <opencv2/core.hpp>

class ObstacleDetector
{
public:
    ObstacleDetector();
    ~ObstacleDetector();
    void detect(cv::Mat frame);
private:
    
};

#endif