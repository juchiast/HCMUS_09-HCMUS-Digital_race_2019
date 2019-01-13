#include "carcontrol.h"
#include "lanedetector.h"
#include "signdetector.h"
CarControl::CarControl()
{
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
    this->commandId = cmdId;
}

void CarControl::driverCar(cv::Mat image)
{
    if (commandId != CommandId::None)
    {
        if (commandId == CommandId::TurnRight)
        {
            int roi_x = image.cols / 3;
            int roi_y = image.rows / 4;
            int roi_width = image.cols - roi_x;
            int roi_height = image.rows - roi_y;
            cv::Rect roi = cv::Rect(roi_x, roi_y, roi_width, roi_height);

            image = image(roi);
        }
    }

    laneDetector->update(image);
    signDetector->detect(image);
    signDetector->visualize(image);
    cv::imshow("View", image);
    cv::waitKey(1);
    driverCar(laneDetector->getLeftLane(), laneDetector->getRightLane(), 50);
}

void CarControl::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity)
{
    int i = left.size() - 11;
    float error = preError;

    if (commandId != CommandId::None)
    {
        doCommand(left, right, velocity);
        return;
    }

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

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
} 

void CarControl::doCommand(const vector<Point> &left, const vector<Point> &right, float velocity)
{
    float error = preError;

    switch (commandId)
    {
        case CommandId::Continue:
        {
            error = 0;
            velocity = 50;
            commandId = CommandId::None;
            break;
        }
        case CommandId::Stop:
        {
            velocity = 0;
            break;
        }
        case CommandId::TurnLeft:
        {
            velocity = 10;
            int i = left.size() - 11;

            while (left[i] == LaneDetector::null) {
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
            break;
        }
        case CommandId::TurnRight:
        {
            velocity = 10;
            int i = right.size() - 11;
            while (right[i] == LaneDetector::null) {
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
            break;
        }
        default:
        {
            return;
        }
    }



    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
}