#include "navigation.hpp"

using namespace cv;

Navigation::Navigation()
{
    carPos.x = 120;
    carPos.y = 300;
}

float Navigation::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x) return 0;
    if (dst.y == carPos.y) return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y; 
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void Navigation::update(const Lane& leftLane, const Lane& rightLane, const std::vector<cds_msgs::SignDetected>& signs)
{
    this->currentSpeed = 50;
    this->currentSteer = 0;
}
    
float Navigation::getSpeed() const
{
    return this->currentSpeed;
}
    
float Navigation::getSteer() const
{
    return this->currentSteer;
}

// void Navigation::turnRight(cv::Mat image, float& error, float& velocity)
// {
//     velocity = 10;

//     laneDetector->update(1, image);
//     std::vector<cv::Point> left, right;

//     left = laneDetector->getLeftLane();
//     right = laneDetector->getRightLane();

//     int i = left.size() - 11;

//     while (left[i] == LaneDetector::null) {
//         i--;
//         if (i < 0) return;
//     }
//     error = errorAngle(left[i] + Point(laneWidth / 2, 0));
//     // error = errorAngle(left[i] + Point(10, 0));

//     currentSpeed = velocity;
//     currentSteer = error;
// }

// void CarControl::turnLeft(cv::Mat image, float& error, float& velocity)
// {
//     velocity = 10;
//     laneDetector->update(2, image);
//     std::vector<cv::Point> left, right;

//     left = laneDetector->getLeftLane();
//     right = laneDetector->getRightLane();

//     int i = right.size() - 11;
//     while (right[i] == LaneDetector::null) {
//         i--;
//         if (i < 0) return;
//     }
//     error = errorAngle(right[i] - Point(laneWidth / 2, 0));
//     // error = errorAngle(right[i] - Point(10, 0));


//     currentSpeed = velocity;
//     currentSteer = error;
// }

// void CarControl::forward(cv::Mat image, float& error, float& velocity)
// {
//     laneDetector->update(0, image);
//     std::vector<cv::Point> left, right;

//     left = laneDetector->getLeftLane();
//     right = laneDetector->getRightLane();

//     int i = left.size() - 11;

//     while (left[i] == LaneDetector::null && right[i] == LaneDetector::null) {
//         i--;
//         if (i < 0) return;
//     }
//     if (left[i] != LaneDetector::null && right[i] !=  LaneDetector::null)
//     {
//         error = errorAngle((left[i] + right[i]) / 2);
//     } 
//     else if (left[i] != LaneDetector::null)
//     {
//         error = errorAngle(left[i] + Point(laneWidth / 2, 0));
//     }
//     else
//     {
//         error = errorAngle(right[i] - Point(laneWidth / 2, 0));
//     }


//     currentSpeed = velocity;
//     currentSteer = error;
// }