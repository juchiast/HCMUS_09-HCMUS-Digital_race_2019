#include "lanedetector.h"

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

void LaneDetector::update(int priority, const Mat &src)
{
    // priority == 1: left, 2: right, 0: forward
    Mat img = preProcess(src);
    vector<Mat> layersV, layersH;
    vector<vector<Point> > pointsV, pointsH;

    layersV = splitLayer(img, VERTICAL);
    pointsV = centerRoadSide(layersV, VERTICAL);

    if (priority != 0)
    {
        cv::Mat lane = Mat::zeros(img.size(), CV_8UC3);
        layersH = splitLayer(img, HORIZONTAL);
        pointsH = centerRoadSide(layersH, HORIZONTAL);

        detectLeftRight(pointsH);
    }

    detectLeftRight(pointsV);

    Mat birdViewVertical, birdViewHorizontal, lane;
    birdViewVertical = Mat::zeros(img.size(), CV_8UC3);
    // birdViewHorizontal = Mat::zeros(img.size(), CV_8UC3);
    lane = Mat::zeros(img.size(), CV_8UC3);

    for (int i = 0; i < pointsV.size(); i++)
     {
        for (int j = 0; j < pointsV[i].size(); j++)
        {
            circle(birdViewVertical, pointsV[i][j], 1, Scalar(0,0,255), 2, 8, 0 );
        }
    }

    imshow("BirdViewVertical", birdViewVertical);

    // for (int i = 0; i < points2.size(); i++)
    //  {
    //     for (int j = 0; j < points2[i].size(); j++)
    //     {
    //         circle(birdViewHorizontal, points2[i][j], 1, Scalar(0,255,0), 2, 8, 0 );
    //     }
    // }

    // imshow("BirdViewHorizontal", birdViewHorizontal);

    for (int i = 1; i < leftLane.size(); i++)
    {
        if (leftLane[i] != null)
        {
            circle(lane, leftLane[i], 1, Scalar(0,0,255), 2, 8, 0 );
        }
    }

    for (int i = 1; i < rightLane.size(); i++)
    {
        if (rightLane[i] != null) {
            circle(lane, rightLane[i], 1, Scalar(255,0,0), 2, 8, 0 );
        }
    }

   imshow("Lane Detect", lane);
}

Mat LaneDetector::preProcess(const Mat &src)
{
    Mat imgThresholded, imgHSV, dst;
    // Mat sobely, sobelyHSV, sobelyThresholded;

    // cv::Sobel(src, sobely, CV_8U, 0, 1, 3);
    // cv::cvtColor(sobely, sobelyHSV, cv::COLOR_RGB2HSV);
    // cv::inRange(sobelyHSV, cv::Scalar(0,0,0), cv::Scalar(180, 255, 170), sobelyThresholded);

    // cv::bitwise_not(sobelyThresholded, sobelyThresholded);

    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]),
        Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]),
        imgThresholded);

    // cv::max(sobelyThresholded, imgThresholded, imgThresholded);


    dst = birdViewTranform(imgThresholded);

//    imshow("Bird View", dst);

    fillLane(dst);

   imshow("Binary", imgThresholded);

    return dst;
}

Mat LaneDetector::laneInShadow(const Mat &src)
{
    Mat shadowMask, shadow, imgHSV, shadowHSV, laneShadow;
    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, Scalar(minShadowTh[0], minShadowTh[1], minShadowTh[2]),
    Scalar(maxShadowTh[0], maxShadowTh[1], maxShadowTh[2]),
    shadowMask);

    src.copyTo(shadow, shadowMask);

    cvtColor(shadow, shadowHSV, COLOR_BGR2HSV);

    inRange(shadowHSV, Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]),
        Scalar(maxLaneInShadow[0], maxLaneInShadow[1], maxLaneInShadow[2]),
        laneShadow);

    return laneShadow;
}

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

vector<Mat> LaneDetector::splitLayer(const Mat &src, int dir)
{
    int rowN = src.rows;
    int colN = src.cols;
    std::vector<Mat> res;

    if (dir == VERTICAL)
    {
        for (int i = 0; i < rowN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(0, i, colN, slideThickness);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    else
    {
        for (int i = 0; i < colN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(i, 0, slideThickness, rowN);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }

    return res;
}

vector<vector<Point> > LaneDetector::centerRoadSide(const vector<Mat> &src, int dir)
{
    vector<std::vector<Point> > res;
    int inputN = src.size();
    for (int i = 0; i < inputN; i++) {
        std::vector<std::vector<Point> > cnts;
        std::vector<Point> tmp;
        findContours(src[i], cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        int cntsN = cnts.size();
        if (cntsN == 0) {
            res.push_back(tmp);
            continue;
        }

        for (int j = 0; j < cntsN; j++) {
            int area = contourArea(cnts[j], false);
            if (area > 10) {
                Moments M1 = moments(cnts[j], false);
                Point2f center1 = Point2f(static_cast<float> (M1.m10 / M1.m00), static_cast<float> (M1.m01 / M1.m00));
                if (dir == VERTICAL) {
                    center1.y = center1.y + slideThickness*i;
                }
                else
                {
                    center1.x = center1.x + slideThickness*i;
                }
                if (center1.x > 0 && center1.y > 0) {
                    tmp.push_back(center1);
                }
            }
        }
        res.push_back(tmp);
    }

    return res;
}

// void LaneDetector::detectLeftRight(const vector<vector<Point> > &points)
// {
//     static vector<Point> lane1, lane2;
//     lane1.clear();
//     lane2.clear();

//     leftLane.clear();
//     rightLane.clear();
//     for (int i = 0; i < BIRDVIEW_HEIGHT / slideThickness; i ++)
//     {
//         leftLane.push_back(null);
//         rightLane.push_back(null);
//     }

//     int pointMap[points.size()][20];
//     int prePoint[points.size()][20];
//     int postPoint[points.size()][20];
//     // int disYPoint[points.size()][20];

//     int disX = 10, disY = 10;
//     int max = -1, max2 = -1;
//     Point2i posMax, posMax2;

//     memset(pointMap, 0, sizeof pointMap);

//     for (int i = 0; i < points.size(); i++)
//     {
//         for (int j = 0; j < points[i].size(); j++)
//         {
//             pointMap[i][j] = 1;
//             prePoint[i][j] = -1;
//             postPoint[i][j] = -1;
//             // disYPoint[i][j] = 0;
//         }
//     }

//     for (int i = points.size() - 2; i >= 0; i--)
//     {
//         for (int j = 0; j < points[i].size(); j++)
//         {
//             int err = 320;
//             for (int m = 1; m < min(points.size() - 1 - i, 5); m++)
//             {
//                 for (int k = 0; k < points[i + 1].size(); k ++)
//                 {
//                     if (abs(points[i + m][k].x - points[i][j].x) < disX &&
//                         abs(points[i + m][k].x - points[i][j].x) < err) {
//                         err = abs(points[i + m][k].x - points[i][j].x);
//                         pointMap[i][j] = pointMap[i + m][k] + 1;
//                         prePoint[i][j] = k;
//                         postPoint[i + m][k] = j;
//                         // disYPoint[i][j] = abs(points[i + m][k].y - points[i][j].y);
//                     }
//                 }
//                 break;
//             }

//             if (pointMap[i][j] > max)
//             {
//                 max = pointMap[i][j];
//                 posMax = Point2i(i, j);
//             }
//         }
//     }

//     for (int i = 0; i < points.size(); i++)
//     {
//         for (int j = 0; j < points[i].size(); j++)
//         {
//             if (pointMap[i][j] > max2 && (i != posMax.x || j != posMax.y) && postPoint[i][j] == -1)
//             {
//                 max2 = pointMap[i][j];
//                 posMax2 = Point2i(i, j);
//             }
//         }
//     }

//     if (max == -1) return;

//     while (max >= 1)
//     {
//         lane1.push_back(points[posMax.x][posMax.y]);
//         if (max == 1) break;

//         posMax.y = prePoint[posMax.x][posMax.y];
//         posMax.x += 1;

//         max--;
//     }

//     while (max2 >= 1)
//     {
//         lane2.push_back(points[posMax2.x][posMax2.y]);
//         if (max2 == 1) break;

//         posMax2.y = prePoint[posMax2.x][posMax2.y];
//         posMax2.x += 1;

//         max2--;
//     }

//     vector<Point> subLane1(lane1.begin(), lane1.begin() + 5);
//     vector<Point> subLane2(lane2.begin(), lane2.begin() + 5);

//     Vec4f line1, line2;

//     fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
//     fitLine(subLane2, line2, 2, 0, 0.01, 0.01);

//     int lane1X = (BIRDVIEW_WIDTH - line1[3]) * line1[0] / line1[1] + line1[2];
//     int lane2X = (BIRDVIEW_WIDTH - line2[3]) * line2[0] / line2[1] + line2[2];

//     if (lane1X < lane2X)
//     {
//         for (int i = 0; i < lane1.size(); i++)
//         {
//             leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
//         }
//         for (int i = 0; i < lane2.size(); i++)
//         {
//             rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
//         }
//     }
//     else
//     {
//         for (int i = 0; i < lane2.size(); i++)
//         {
//             leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
//         }
//         for (int i = 0; i < lane1.size(); i++)
//         {
//             rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
//         }
//     }
// }

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
                        // disYPoint[i][j] = abs(points[i + m][k].y - points[i][j].y);
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

    // for (int i = 0; i < points.size(); i++)
    // {
    //     std::sort(points[i].begin(), points[i].end(), [](const Point& first, const Point& last) -> bool {
    //         return first.x > last.x;
    //     });

    //     for (int j = 0; j < points[i].size() - 1; j++)
    //     {
    //         if (points[i][j + 1].x - points[i][j].x < disX)
    //         {
    //             prePoint[i][j] = 
    //         }
            
    //     }
    // }

    while (max2 >= 1)
    {
        lane2.push_back(points[posMax2.x][posMax2.y]);
        if (max2 == 1) break;

        posMax2.y = prePoint[posMax2.x][posMax2.y];
        posMax2.x += 1;

        max2--;
    }

    vector<Point> subLane1(lane1.begin(), lane1.begin() + 5);
    vector<Point> subLane2(lane2.begin(), lane2.begin() + 5);

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

