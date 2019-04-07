#include <ros/ros.h>

#include "cds_msgs/Lane.h"
#include "cds_msgs/SignDetected.h"
#include "cds_msgs/System.h"
#include "cds_msgs/Object.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include "navigation.hpp"

static Navigation::Lane leftLane, rightLane;
static Navigation::TurningFlags leftTurning, rightTurning;

static ros::Publisher speedPublisher, steerPublisher;
static ros::Publisher changeSpeedPublisher;

static Navigation navigation;
static bool isStop = false;

static bool isIncBtnPressed = false;
static bool isDecBtnPressed = false;

static bool shouldTurn = false;
static constexpr const int DIRECTION_LEFT = 1;
static constexpr const int DIRECTION_RIGHT = -1;
static int turnDirection = 0;

static void publishSpeedChange()
{
    static char buffer[50];
    snprintf(buffer, 50, "1:1:DEF_VELOCITY %3d", Navigation::DEF_VELOCITY);
    std_msgs::String msg;
    msg.data = buffer;
    changeSpeedPublisher.publish(msg);
}

static void increaseSpeedCallback(const std_msgs::Bool& msg)
{
    if (msg.data == true)
    {
        isIncBtnPressed = true;
    }
    else
    {
        if (isIncBtnPressed == true)
        {
            // inc speed
            Navigation::MIN_VELOCITY += 1;
            Navigation::DEF_VELOCITY += 1;
            Navigation::MAX_VELOCITY += 1;
            publishSpeedChange();
        }
        isIncBtnPressed = false;
    }
}

static void decreaseSpeedCallback(const std_msgs::Bool& msg)
{
    if (msg.data == true)
    {
        isDecBtnPressed = true;
    }
    else
    {
        if (isDecBtnPressed == true)
        {
            // dec speed
            Navigation::MIN_VELOCITY -= 1;
            Navigation::DEF_VELOCITY -= 1;
            Navigation::MAX_VELOCITY -= 1;
            publishSpeedChange();
        }
        isDecBtnPressed = false;
    }
}

void convertLandMarkMsg2Lane(const std::vector<cds_msgs::LandMark> &landmarkMsg, Navigation::Lane &lane)
{
    lane.clear();
    for (size_t i = 0; i < landmarkMsg.size(); i++)
    {
        const cds_msgs::LandMark &landmark = landmarkMsg[i];
        lane.push_back(cv::Point{landmark.x, landmark.y});
    }
}

void convertTurningMsg2Turning(const std::vector<std_msgs::Bool> &turningMsg, Navigation::TurningFlags &turning)
{
    turning.clear();
    for (const std_msgs::Bool &msg : turningMsg)
    {
        turning.push_back(msg.data);
    }
}

static void publishSteer(float steer)
{
    std_msgs::Float32 steerMsg;
    steerMsg.data = steer;
    steerPublisher.publish(steerMsg);
}

static void publishSpeed(float speed)
{
    std_msgs::Float32 speedMsg;
    speedMsg.data = speed;
    speedPublisher.publish(speedMsg);
}

static void laneCallback(const cds_msgs::LaneConstPtr &laneMsg)
{
    auto &&msgLeftLane = laneMsg->leftLane;
    auto &&msgRightLane = laneMsg->rightLane;
    auto &&msgLeftTurning = laneMsg->leftTurn;
    auto &&msgRightTurning = laneMsg->rightTurn;

    convertLandMarkMsg2Lane(msgLeftLane, leftLane);
    convertLandMarkMsg2Lane(msgRightLane, rightLane);
    convertTurningMsg2Turning(msgLeftTurning, leftTurning);
    convertTurningMsg2Turning(msgRightTurning, rightTurning);

    navigation.update(leftLane, rightLane);
    navigation.update(leftTurning, rightTurning);
}

static bool isTurn(const cds_msgs::SignDetected& signMsg)
{
    return signMsg.width * signMsg.height > 3800;
}

static void signCallback(const cds_msgs::SignDetected &signMsg)
{
    shouldTurn = isTurn(signMsg);
    if (shouldTurn)
    {
        turnDirection = (signMsg.id == 1) ? DIRECTION_LEFT : DIRECTION_RIGHT;
    }
    SignDetected sign;
    sign.id = signMsg.id;
    sign.confident = signMsg.confident;
    sign.height = signMsg.height;
    sign.width = signMsg.width;
    sign.x = signMsg.x;
    sign.y = signMsg.y;
    navigation.update(sign);
}

static void objCallback(const cds_msgs::Object &msg)
{
    navigation.updateObjectDirection(msg.direction);
}

static void publishWhenTurning()
{
    static const float SPEED_TURN = 0.5 * 8 * 5 / 18.0f; // m/s
    static const int ANGLE = 90;    // Degree
    static const float DISTANCE_TURN = 1;   // m
    float angleTurn = ANGLE * SPEED_TURN / DISTANCE_TURN; 
    publishSpeed(8);

    angleTurn = angleTurn * turnDirection;
    publishSteer(angleTurn);

    ROS_INFO("angle %f, time %f, direct %d", angleTurn, 1 / (SPEED_TURN / DISTANCE_TURN), turnDirection);

    //ros::Rate sleepRate{};// SPEED_TURN / DISTANCE_TURN};
    ros::Duration(DISTANCE_TURN / SPEED_TURN).sleep();
    ROS_INFO("WAKEUP");
    shouldTurn = false;
}

static void systemCallback(const cds_msgs::System &systemMsg)
{
    isStop = systemMsg.isStop.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation");

    ros::NodeHandle nh;

    ros::Subscriber laneSub = nh.subscribe("/lane_detected", 1, laneCallback);
    ros::Subscriber signSub = nh.subscribe("/sign_detected", 1, signCallback);
    ros::Subscriber objSub = nh.subscribe("/object_detected", 1, objCallback);
    ros::Subscriber systemSub = nh.subscribe("/system", 1, systemCallback);
    ros::Subscriber decSpeedSub = nh.subscribe("/bt2_status", 10, increaseSpeedCallback);
    ros::Subscriber incSpeedSub = nh.subscribe("/bt3_status", 10, decreaseSpeedCallback);

    Navigation::DEF_VELOCITY = nh.param("/navigation/DEF_VELOCITY", 8);
    Navigation::MIN_VELOCITY = nh.param("/navigation/MIN_VELOCITY", 5);
    Navigation::MAX_VELOCITY = nh.param("/navigation/MAX_VELOCITY", 30);

    ROS_INFO("DEF_VELOCITY = %d", Navigation::DEF_VELOCITY);
    ROS_INFO("MIN_VELOCITY = %d", Navigation::MIN_VELOCITY);
    ROS_INFO("MAX_VELOCITY = %d", Navigation::MAX_VELOCITY);

    speedPublisher = nh.advertise<std_msgs::Float32>("/set_speed_car_api", 10);
    steerPublisher = nh.advertise<std_msgs::Float32>("/set_steer_car_api", 10);
    changeSpeedPublisher = nh.advertise<std_msgs::String>("/lcd_print", 10);
    ROS_INFO("navigation node started");

    ros::Rate rate{15};
    while (ros::ok())
    {
        ros::spinOnce();

        if (isStop)
        {
            publishSpeed(0);
            publishSteer(0);
        } else 
        // if (shouldTurn)
        {
        //     publishWhenTurning();
        // } else 
        // {
            publishSpeed(navigation.getSpeed());
            publishSteer(navigation.getSteer());
            if (shouldTurn)
            {
                ros::Duration(0.3f).sleep();
            }
            // navigation.visualize();
        }


        rate.sleep();
    }


    publishSpeed(0);
    publishSteer(0);

    return 0;
}