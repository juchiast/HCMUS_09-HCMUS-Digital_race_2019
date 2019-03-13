#include "carcontrol.h"
#include "lanedetector.h"
#include "signdetector.h"

#include <std_msgs/Float32.h>
#include <cmath>


CarControl::CarControl()
{
    laneDetector = new LaneDetector();
    signDetector = new SignDetector();
    carPos.x = 120;
    carPos.y = 300;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("/set_speed_car_api", 10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("/set_steer_car_api", 10);
}

CarControl::~CarControl() 
{
    delete laneDetector;
    delete signDetector;
}

float CarControl::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x) return 0;
    if (dst.y == carPos.y) return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y; 
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void CarControl::drive(cv::Mat image)
{
    float error = 0;
    float velocity = this->lastVelocity;

    laneDetector->detect(image);
    signDetector->detect(image);

    auto&& signs = signDetector->getDetections();
    if (signs.size() > 0)
    {
        signDetector->visualize(image);
        // find max conf
        // auto&& maxConfIter = std::max_element(signs.begin(), signs.end(), [](const Sign& a, const Sign& b) -> bool {
        //     return (a.confident > b.confident);
        // });

        // int sign = maxConfIter->id;

        // if (sign == TrafficSign::Left) {
        //     this->turnLeft(image, error, velocity);
        // } else if (sign == TrafficSign::Right) {
        //     this->turnRight(image, error, velocity);
        // } else if (sign == 0) {
        //     velocity = 10; // slow down
            this->forward(image, error, velocity);
        // }
    } else {
        velocity = 50;
        this->forward(image, error, velocity);
    }

    publishSpeed(velocity);
    publishSteer(error);

   cv::waitKey(1);

}

void CarControl::turnRight(cv::Mat image, float& error, float& velocity)
{
    velocity = 10;

    laneDetector->detect(image);
    std::vector<cv::Point> left, right;

    left = laneDetector->getLeftLane();
    right = laneDetector->getRightLane();

    int i = left.size() - 11;

    while (left[i] == LaneDetector::null) {
        i--;
        if (i < 0) return;
    }
    error = errorAngle(left[i] + Point(laneWidth / 2, 0));
    // error = errorAngle(left[i] + Point(10, 0));
}

void CarControl::turnLeft(cv::Mat image, float& error, float& velocity)
{
    velocity = 10;
    laneDetector->detect(image);
    std::vector<cv::Point> left, right;

    left = laneDetector->getLeftLane();
    right = laneDetector->getRightLane();

    int i = right.size() - 11;
    while (right[i] == LaneDetector::null) {
        i--;
        if (i < 0) return;
    }
    error = errorAngle(right[i] - Point(laneWidth / 2, 0));
    // error = errorAngle(right[i] - Point(10, 0));
}

void CarControl::forward(cv::Mat image, float& error, float& velocity)
{
    laneDetector->detect(image);
    std::vector<cv::Point> left, right;

    left = laneDetector->getLeftLane();
    right = laneDetector->getRightLane();

    int i = left.size() - 11;

    while (left[i] == LaneDetector::null && right[i] == LaneDetector::null) {
        i--;
        if (i < 0) return;
    }
    if (left[i] != LaneDetector::null && right[i] !=  LaneDetector::null)
    {
        error = errorAngle((left[i] + right[i]) / 2);
    } 
    else if (left[i] != LaneDetector::null)
    {
        error = errorAngle(left[i] + Point(laneWidth / 2, 0));
    }
    else
    {
        error = errorAngle(right[i] - Point(laneWidth / 2, 0));
    }
}

void CarControl::stop()
{
    publishSpeed(0);
    publishSteer(0);
}

void CarControl::publishSpeed(const float& velocity)
{
    std_msgs::Float32 speed;
    speed.data = velocity;
    speed_publisher.publish(speed);   
}

void CarControl::publishSteer(const float& angle)
{
    std_msgs::Float32 steer;
    steer.data = angle;
    steer_publisher.publish(steer);
}