#ifndef CARCONTROL_H
#define CARCONTROL_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>


using namespace std;
using namespace cv;

class LaneDetector;
class SignDetector;

enum CommandId
{
    None = 0,
    TurnRight = 1,
    TurnLeft = 2,
    Stop = 3,           // Test only
    Continue = 4        // Test only
};

class CarControl 
{
public:
    CarControl();
    ~CarControl();
    void receiveCommand(CommandId commandId);
    void driverCar(cv::Mat image);

private:
    void turnLeft(cv::Mat image, float& error, float& velocity);
    void turnRight(cv::Mat image, float& error, float& velocity);
    void forward(cv::Mat image, float& error, float& velocity);



    float errorAngle(const Point &dst);


private:
    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;
    
    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;

    Point carPos;

    float laneWidth = 50;

    float lastVelocity = 40;
    float minVelocity = 10;
    float maxVelocity = 50;

    float preError;

    int commandId;
    bool isTurning;
    LaneDetector* laneDetector;
    SignDetector* signDetector;
};

#endif
