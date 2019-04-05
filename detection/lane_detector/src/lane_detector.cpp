#include "lane_detector.hpp"
#include <algorithm>

using namespace cv;
using namespace std;

int LaneDetector::slideThickness = 10;
int LaneDetector::BIRDVIEW_WIDTH = 240;
int LaneDetector::BIRDVIEW_HEIGHT = 320;
int LaneDetector::SKYLINE = 120;
int LaneDetector::BIRDVIEW_BOTTOM_DELTA = 50;
static int depthLow = 700;
static int depthHigh = 1500;

Point LaneDetector::null = Point();

LaneDetector::LaneDetector() {
    colorImage = cv::Mat::zeros(cv::Size(640,480), CV_8UC3);
    depthImage = cv::Mat::zeros(cv::Size(640,480), CV_16UC1);
    cv::namedWindow("config");

    cv::createTrackbar("depthLow", "config", &depthLow, 5000);
    cv::createTrackbar("depthHigh", "config", &depthHigh, 5000);
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

vector<bool> LaneDetector::getLeftTurn()
{
    return leftTurn;
}

vector<bool> LaneDetector::getRightTurn()
{
    return rightTurn;
}

void LaneDetector::updateColorImage(const Mat &colorImage){
    this->colorImage = colorImage;
}

void LaneDetector::updateDepthImage(const Mat &depthImage)
{
    this->depthImage = depthImage;
}

void LaneDetector::detect()
{
    // cv::Mat depthMask;
    // cv::inRange(depthImage, cv::Scalar(0), cv::Scalar(2000), depthMask); 
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    // cv::Mat morph;
    // cv::morphologyEx(depthMask, morph, cv::MORPH_CLOSE, kernel);
    // depthMask = morph;

    // depthMask(cv::Rect(depthMask.cols-30, 0, 30, depthMask.rows * 0.8)) = cv::Scalar(0);
    // depthMask(cv::Rect(0, 0, 30, depthMask.rows * 0.8)) = cv::Scalar(0);

    // cv::Mat filteredImage;
    // colorImage.copyTo(filteredImage, depthMask);
    // cv::imshow("Filtered", filteredImage);
    
    Mat img = preProcess(colorImage);

    cv::Mat visualization;
    cv::cvtColor(img, visualization, cv::COLOR_GRAY2BGR);

    vector<Mat> layers = splitLayer(img);
    

    vector<vector<Point>> centroidPoints = findLayerCentroids(layers);
    visualizeCentroids(visualization, centroidPoints);

    detectLeftRight(visualization, centroidPoints);

    // post processing
    int numPoints = BIRDVIEW_HEIGHT / slideThickness;
    if (countNonNull(leftLane) < 5)
    {
        leftLane = vector<cv::Point>(numPoints, null); // it should mark as lost
    }
    if (countNonNull(rightLane) < 5)
    {
        rightLane = vector<cv::Point>(numPoints, null);
    }

    if ((isLaneNull(leftLane) && isRightCurve(rightLane)) || isLaneNull(rightLane) && isLeftCurve(leftLane))
    {
        std::swap(leftLane, rightLane);
    }

    visualizeLanes(visualization);


    leftTurn = findTurnable(leftLane, visualization);
    rightTurn = findTurnable(rightLane, visualization);


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
    imshow("Binary", imgThresholded);

    cv::imshow("DepthImage", depthImage);

    cv::Mat depthThreshold;
    inRange(depthImage, Scalar(depthLow), Scalar(depthHigh), depthThreshold);
    int height = 0.7 * depthThreshold.rows;
    depthThreshold(cv::Rect(0, height, depthThreshold.cols, depthThreshold.rows - height)) = Scalar(0);
    // cv::imshow("DepthThreshold", depthThreshold);

    cv::Mat filtered;
    colorImage.copyTo(filtered, depthThreshold);
    // cv::imshow("Filtered", filtered);


    // cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));
    // cv::Mat skel = cv::Mat::zeros(imgThresholded.size(), CV_8UC1);
    // bool done;
    // cv::Mat temp;
    // do
    // {
    //     cv::morphologyEx(imgThresholded, temp, cv::MORPH_OPEN, element);
    //     cv::bitwise_not(temp, temp);
    //     cv::bitwise_and(imgThresholded, temp, temp);
    //     cv::bitwise_or(skel, temp, skel);
    //     cv::erode(imgThresholded, imgThresholded, element);
    //     double max = 0;
    //     cv::minMaxLoc(imgThresholded, 0, &max);
    //     done = (max == 0);
    // } while (!done);
    
    // cv::imshow("Skeleton", skel);

    dst = birdViewTranform(imgThresholded);
    cv::Mat depthBirdview = birdViewTranform(depthImage);
    cv::imshow("DepthBirdview", depthBirdview);
    // dst = imgThresholded;

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
            // cv::Rect bounding = cv::boundingRect(contour);
            // int area = contourArea(contour, false);
            // if (area > 10) {
            // float ratio = bounding.width * 1.0f / bounding.height;
            // if (bounding.area() > 10 && ratio > 0.8 && ratio < 1.8)
            {
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

void LaneDetector::detectLeftRight(cv::Mat visualization, const vector<vector<Point> > &points)
{
    static vector<Point> lane1, lane2;
    lane1.clear();
    lane2.clear();

    leftLane.clear();
    rightLane.clear();

    int numPoints = BIRDVIEW_HEIGHT / slideThickness;
    leftLane = vector<cv::Point>(numPoints, null);
    rightLane = vector<cv::Point>(numPoints, null);
    leftTurn = vector<bool>(numPoints, false);
    rightTurn = vector<bool>(numPoints, false);

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
            for (int m = 1; m < std::min(points.size() - 1 - i, 5UL); m++)
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

    vector<Point> subLane1(lane1.end() - 5, lane1.end());
    vector<Point> subLane2(lane2.end() - 5, lane2.end());

    Vec4f line1, line2;

    fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
    fitLine(subLane2, line2, 2, 0, 0.01, 0.01);

    // std::cout << "Line 1: " << line1 << std::endl;
    // std::cout << "Line 2: " << line2 << std::endl;


    // int lane1X = (BIRDVIEW_WIDTH - line1[3]) * line1[0] / line1[1] + line1[2];
    // int lane2X = (BIRDVIEW_WIDTH - line2[3]) * line2[0] / line2[1] + line2[2];
    int lane1X = line1[2];
    int lane2X = line2[2];

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

bool LaneDetector::isLaneNull(const LanePoint& points) const
{
    return std::all_of(points.begin(), points.end(), [](const cv::Point& point) {
        return point == null;
    });
}

std::vector<bool> LaneDetector::findTurnable(const std::vector<cv::Point>& lane, cv::Mat visualization)
{
    std::vector<bool> result(lane.size(), false);
    int threshold = 1;
	for(size_t i = lane.size() - 1; i > 1; i--)
	{
		int deltaX = lane[i].x - lane[i+1].x;
		if (abs(deltaX) > threshold)
		{
            result[i] = true;
			// cv::circle(visualization, lane[i], 2, cv::Scalar{0,255,255}, -1);
		}
	}
    return result;
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

    src_vertices[0] = Point(0, SKYLINE);
    src_vertices[1] = Point(width, SKYLINE);
    src_vertices[2] = Point(width, height);
    src_vertices[3] = Point(0, height);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(BIRDVIEW_WIDTH, 0);
    dst_vertices[2] = Point(BIRDVIEW_WIDTH - BIRDVIEW_BOTTOM_DELTA, BIRDVIEW_HEIGHT);
    dst_vertices[3] = Point(BIRDVIEW_BOTTOM_DELTA, BIRDVIEW_HEIGHT);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);

    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    return dst;
}

int LaneDetector::countNonNull(const LanePoint& points) const
{
    int count = 0;
    for (size_t i = 0; i < points.size(); i++)
    {
        if (points[i] != null)
        {
            count++;
        }
    }
    return count;
}

bool LaneDetector::isLeftCurve(const LanePoint& points) const
{
    int size = countNonNull(points);
    if (size < 5)
    {
        return false;
    }

    cv::Point prevPoint, curPoint;
    int countDelta = 0;
    for (int i = leftLane.size() - 1; i > 0; i--)
    {
        if (leftLane[i] != null)
        {
            prevPoint = curPoint;
            curPoint = leftLane[i];
        }
        
        if (curPoint != null && prevPoint != null && (curPoint.x - prevPoint.x) < 0)
        {
            countDelta++;
        }
    }
    
    if (countDelta * 1.0f / size > 0.5)
    {
        return true;
    }

    return false;
}

bool LaneDetector::isRightCurve(const LanePoint& points) const
{
    int size = countNonNull(points);
    if (size < 5)
    {
        return false;
    }

    cv::Point prevPoint, curPoint;
    int countDelta = 0;
    for (int i = leftLane.size() - 1; i > 0; i--)
    {
        if (leftLane[i] != null)
        {
            prevPoint = curPoint;
            curPoint = leftLane[i];
        }
        
        if (curPoint != null && prevPoint != null && (curPoint.x - prevPoint.x) < 0)
        {
            countDelta++;
        }
    }
    
    if (countDelta * 1.0f / size > 0.5)
    {
        return true;
    }

    return false;
}