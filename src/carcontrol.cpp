#include "carcontrol.h"
#include "lanedetector.h"
#include "signdetector.h"

CarControl::CarControl()
{
    isTurning = false;
    laneDetector = new LaneDetector();
    signDetector = new SignDetector();
    commandId = 0;
    carPos.x = 120;
    carPos.y = 300;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("team405_steerAngle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("team405_speed",10);
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

void CarControl::receiveCommand(CommandId cmdId)
{
    if (cmdId == CommandId::Stop)
    {
        this->lastVelocity = 0;
    } else if (cmdId == CommandId::Continue)
    {
        this->lastVelocity = 50;
    }
}

void CarControl::driverCar(cv::Mat image)
{
    float error = 0;
    float velocity = this->lastVelocity;

    // if (signDetector->detect(image)) {
        // TrafficSign sign = signDetector->getTrafficSign();

        // if (sign == TrafficSign::Left) {
            // this->turnLeft(image, error, velocity);
        // } else if (sign == TrafficSign::Right) {
            // this->turnRight(image, error, velocity);
        // } else if (sign == 0) {
            // velocity = 10; // slow down
            // this->forward(image, error, velocity);
        // }
        // signDetector->visualize(image);
    // } else {
        velocity = 50;
        this->forward(image, error, velocity);
    // }

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);   

    lastVelocity = velocity;

    cv::imshow("View", image);
    cv::waitKey(1);
}

void CarControl::turnRight(cv::Mat image, float& error, float& velocity)
{
    velocity = 10;

    laneDetector->update(1, image);
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
    laneDetector->update(2, image);
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
    laneDetector->update(0, image);
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