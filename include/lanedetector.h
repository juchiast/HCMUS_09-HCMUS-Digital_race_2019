#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

class LaneDetector
{
public:
    LaneDetector();
    ~LaneDetector();

    void detect(cv::Mat src);
    
    vector<Point> getLeftLane();
    vector<Point> getRightLane();

    static int slideThickness;

    static int BIRDVIEW_WIDTH;
    static int BIRDVIEW_HEIGHT;

    static int VERTICAL;
    static int HORIZONTAL;

    static Point null; // 

private:
    Mat preProcess(const Mat &src);

    Mat morphological(const Mat &imgHSV);
    Mat birdViewTranform(const Mat &source);
    void fillLane(Mat &src);
    vector<Mat> splitLayer(const Mat &src, int dir = VERTICAL);
    vector<vector<Point> > centerRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    // bool detectLeftRight(const Point& currentPos, const vector<vector<Point> > &points);
    void detectLeftRight(const vector<vector<Point>>& points);
    Mat laneInShadow(const Mat &src);

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 65, 255}; // 179, 30, 255
    int minShadowTh[3] = {90, 43, 36};
    int maxShadowTh[3] = {120, 81, 171};
    int minLaneInShadow[3] = {90, 43, 97};
    int maxLaneInShadow[3] = {120, 80, 171};
    int binaryThreshold = 180;

    int skyLine = 85;
    int shadowParam = 40;

    vector<Point> leftLane, rightLane;

    // Invert Perspective Transform frame & params
    Mat invertBirdViewTransform(const Mat& birdview);
    cv::Mat topViewFrame, M, invM;
};

#endif
