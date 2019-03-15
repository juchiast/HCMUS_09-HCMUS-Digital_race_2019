#include "lane_detector.hpp"

using namespace cv;
using namespace std;

int min(int a, int b)
{
    return a < b ? a : b;
}

int LaneDetector::slideThickness = 10;
int LaneDetector::BIRDVIEW_WIDTH = 240;
int LaneDetector::BIRDVIEW_HEIGHT = 320;
int LaneDetector::VERTICAL = 0;
int LaneDetector::HORIZONTAL = 1;
Point LaneDetector::null = Point();

LaneDetector::LaneDetector() {
    // cvCreateTrackbar("LowH", "Threshold", &minThreshold[0], 179);
    // cvCreateTrackbar("HighH", "Threshold", &maxThreshold[0], 179);

    // cvCreateTrackbar("LowS", "Threshold", &minThreshold[1], 255);
    // cvCreateTrackbar("HighS", "Threshold", &maxThreshold[1], 255);

    // cvCreateTrackbar("LowV", "Threshold", &minThreshold[2], 255);
    // cvCreateTrackbar("HighV", "Threshold", &maxThreshold[2], 255);

    // cvCreateTrackbar("Shadow Param", "Threshold", &shadowParam, 255);
}

LaneDetector::~LaneDetector(){}

vector<Point> LaneDetector::getLeftLane()
{
    return leftLane;
}

vector<Point> LaneDetector::getRightLane()
{
    return rightLane;
}

void LaneDetector::detect(const Mat &src)
{
    // cv::Mat visualization = cv::Mat::zeros(src.size(), CV_8UC3);

    Mat img = preProcess(src);

    vector<Mat> planes;

    for(int i=0;i<3;i++)
        planes.push_back(img);

    cv::Mat visualization;
    merge(planes, visualization);

    vector<Mat> layers = splitLayer(img);
    

    vector<vector<Point>> centroidPoints = findLayerCentroids(layers);
    visualizeCentroids(visualization, centroidPoints);

    detectLeftRight(centroidPoints);

    int laneThreshold = 15;
    if (leftLane.size() < laneThreshold)
    {
        for (int i = 0; i < leftLane.size(); i++)
        {
            leftLane[i] = null;
        }
    }

    if (rightLane.size() < laneThreshold)
    {
        for (int i = 0; i < rightLane.size(); i++)
        {
            rightLane[i] = null;
        }
    }


    visualizeLanes(visualization);

    cv::imshow("Lane detection", visualization);
    cv::waitKey(1);
}

Mat LaneDetector::preProcess(const Mat &src)
{
    Mat imgThresholded, imgHSV, dst;

    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, 
        Scalar(minThreshold[0], minThreshold[1], minThreshold[2]),
        Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]),
        imgThresholded);

    dst = birdViewTranform(imgThresholded);

    fillLane(dst);

    return dst;
}

// Mat LaneDetector::laneInShadow(const Mat &src)
// {
//     Mat shadowMask, shadow, imgHSV, shadowHSV, laneShadow;
//     cvtColor(src, imgHSV, COLOR_BGR2HSV);

//     inRange(imgHSV, Scalar(minShadowTh[0], minShadowTh[1], minShadowTh[2]),
//     Scalar(maxShadowTh[0], maxShadowTh[1], maxShadowTh[2]),
//     shadowMask);

//     src.copyTo(shadow, shadowMask);

//     cvtColor(shadow, shadowHSV, COLOR_BGR2HSV);

//     inRange(shadowHSV, Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]),
//         Scalar(maxLaneInShadow[0], maxLaneInShadow[1], maxLaneInShadow[2]),
//         laneShadow);

//     return laneShadow;
// }

void LaneDetector::fillLane(Mat &src)
{
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI/180, 1);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, CV_AA);
    }
}

vector<Mat> LaneDetector::splitLayer(const Mat &src)
{
    int rowN = src.rows;
    int colN = src.cols;
    std::vector<Mat> res;

    for (int i = 0; i < rowN - slideThickness; i += slideThickness) {
        Mat tmp;
        Rect crop(0, i, colN, slideThickness);
        tmp = src(crop);
        res.push_back(tmp);
    }

    return res;
}

std::vector<std::vector<cv::Point> > LaneDetector::findLayerCentroids(const std::vector<cv::Mat>& layers)
{
    vector<std::vector<Point>> res;
    for (int i = 0; i < layers.size(); i++) {
        const cv::Mat& currentLayer = layers[i];
        std::vector<std::vector<Point>> contours;
        std::vector<Point> layerCentroids;
        findContours(currentLayer, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

        if (contours.empty())
        {
            res.push_back({});
            continue;
        }

        for (const auto& contour : contours)
        {
            int area = contourArea(contour, false);
            if (area > 10) {
                Moments M1 = moments(contour, false);
                Point2f centroid = Point2f(static_cast<float> (M1.m10 / M1.m00), static_cast<float> (M1.m01 / M1.m00));
                centroid.y += slideThickness * i;
                if (centroid.x > 0 && centroid.y > 0) {
                    layerCentroids.push_back(centroid);
                }
            }
        }
        res.push_back(layerCentroids);
    }

    return res;
}

void LaneDetector::detectLeftRight(const vector<vector<Point> > &points)
{
    static vector<Point> lane1, lane2;
    lane1.clear();
    lane2.clear();

    leftLane.clear();
    rightLane.clear();
    for (int i = 0; i < BIRDVIEW_HEIGHT / slideThickness; i ++)
    {
        leftLane.push_back(null);
        rightLane.push_back(null);
    }

    int pointMap[points.size()][20];
    int prePoint[points.size()][20];
    int postPoint[points.size()][20];

    int disX = 10, disY = 10;
    int max = -1, max2 = -1;
    Point2i posMax, posMax2;

    memset(pointMap, 0, sizeof pointMap);

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }



    for (int i = points.size() - 2; i >= 0; i--)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            int err = 320;
            for (int m = 1; m < min(points.size() - 1 - i, 5); m++)
            {
                for (int k = 0; k < points[i + 1].size(); k ++)
                {
                    if (abs(points[i + m][k].x - points[i][j].x) < disX &&
                        abs(points[i + m][k].x - points[i][j].x) < err) {
                        err = abs(points[i + m][k].x - points[i][j].x);
                        pointMap[i][j] = pointMap[i + m][k] + 1;
                        prePoint[i][j] = k;
                        postPoint[i + m][k] = j;
                    }
                }
                break;
            }
            
            if (pointMap[i][j] > max)
            {
                max = pointMap[i][j];
                posMax = Point2i(i, j);
            }
        }
    }

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max2 && (i != posMax.x || j != posMax.y) && postPoint[i][j] == -1)
            {
                max2 = pointMap[i][j];
                posMax2 = Point2i(i, j);
            }
        }
    }

    if (max == -1) return;

    while (max >= 1)
    {
        lane1.push_back(points[posMax.x][posMax.y]);
        if (max == 1) break;

        posMax.y = prePoint[posMax.x][posMax.y];
        posMax.x += 1;

        max--;
    }

    while (max2 >= 1)
    {
        lane2.push_back(points[posMax2.x][posMax2.y]);
        if (max2 == 1) break;

        posMax2.y = prePoint[posMax2.x][posMax2.y];
        posMax2.x += 1;

        max2--;
    }

    vector<Point> subLane1(lane1.begin(), lane1.begin() + 3);
    vector<Point> subLane2(lane2.begin(), lane2.begin() + 3);

    Vec4f line1, line2;

    fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
    fitLine(subLane2, line2, 2, 0, 0.01, 0.01);

    int lane1X = (BIRDVIEW_WIDTH - line1[3]) * line1[0] / line1[1] + line1[2];
    int lane2X = (BIRDVIEW_WIDTH - line2[3]) * line2[0] / line2[1] + line2[2];

    if (lane1X < lane2X)
    {
        for (int i = 0; i < lane1.size(); i++)
        {
            leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
        for (int i = 0; i < lane2.size(); i++)
        {
            rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
    }
    else
    {
        for (int i = 0; i < lane2.size(); i++)
        {
            leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
        for (int i = 0; i < lane1.size(); i++)
        {
            rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
    }
}

void LaneDetector::visualizeCentroids(cv::Mat visualizeImage, const vector<vector<Point>>& centroids)
{
    const cv::Scalar centroidColor = cv::Scalar(255, 255, 0);

    for (int i = 0; i < centroids.size(); i++)
     {
        for (int j = 0; j < centroids[i].size(); j++)
        {
            circle(visualizeImage, centroids[i][j], 1, centroidColor, 2, 8, 0);
        }
    }
}

void LaneDetector::visualizeLanes(cv::Mat visualizeImage)
{
    const cv::Scalar leftLaneColor{0,0,255};
    const cv::Scalar rightLaneColor{255,0,0};

    for (int i = 1; i < leftLane.size(); i++)
    {
        if (leftLane[i] != null)
        {
            circle(visualizeImage, leftLane[i], 1, leftLaneColor, 2, 8, 0 );
        }
    }

    for (int i = 1; i < rightLane.size(); i++)
    {
        if (rightLane[i] != null) {
            circle(visualizeImage, rightLane[i], 1, rightLaneColor, 2, 8, 0 );
        }
    }
}


Mat LaneDetector::birdViewTranform(const Mat &src)
{
    Point2f src_vertices[4];

    int width = src.size().width;
    int height = src.size().height;

    src_vertices[0] = Point(0, skyLine);
    src_vertices[1] = Point(width, skyLine);
    src_vertices[2] = Point(width, height);
    src_vertices[3] = Point(0, height);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(BIRDVIEW_WIDTH, 0);
    dst_vertices[2] = Point(BIRDVIEW_WIDTH - 105, BIRDVIEW_HEIGHT);
    dst_vertices[3] = Point(105, BIRDVIEW_HEIGHT);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);

    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    return dst;
}