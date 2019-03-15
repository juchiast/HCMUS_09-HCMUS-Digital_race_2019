#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <algorithm>



class LaneDetector
{
public:
    LaneDetector();
    ~LaneDetector();

    void detect(const cv::Mat &src);
    
    std::vector<cv::Point> getLeftLane();
    std::vector<cv::Point> getRightLane();

    static int slideThickness;

    static int BIRDVIEW_WIDTH;
    static int BIRDVIEW_HEIGHT;

    static int VERTICAL;
    static int HORIZONTAL;

    static cv::Point null; 

private:
    cv::Mat preProcess(const cv::Mat &src);
    void fillLane(cv::Mat &src);
    cv::Mat birdViewTranform(const cv::Mat &source);
    std::vector<cv::Mat> splitLayer(const cv::Mat &src);

    std::vector<std::vector<cv::Point> > findLayerCentroids(const std::vector<cv::Mat> &src);

    void detectLeftRight(const std::vector<std::vector<cv::Point>>& points);

    void visualizeCentroids(cv::Mat visualizeImage, const std::vector<std::vector<cv::Point>>& centroids);
    void visualizeLanes(cv::Mat visualizeImage);

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 65, 255}; // 179, 30, 255
    int minShadowTh[3] = {90, 43, 36};
    int maxShadowTh[3] = {120, 81, 171};
    int minLaneInShadow[3] = {90, 43, 97};
    int maxLaneInShadow[3] = {120, 80, 171};
    int binaryThreshold = 180;

    int skyLine = 85;
    int shadowParam = 40;

    std::vector<cv::Point> leftLane, rightLane;
};

#endif
