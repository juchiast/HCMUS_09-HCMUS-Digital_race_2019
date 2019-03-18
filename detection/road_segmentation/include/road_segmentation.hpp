#ifndef ROAD_SEGMENTATION_HPP
#define ROAD_SEGMENTATION_HPP

#include <opencv2/core.hpp>

class RoadSegmentation
{
public:
    void detect(cv::Mat frame);
    cv::Mat getSegmentation() const;
private:
    cv::Mat binary;
};



#endif