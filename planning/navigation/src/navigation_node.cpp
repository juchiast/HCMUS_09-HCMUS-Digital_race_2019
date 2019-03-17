#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>

#include "cds_msgs/Lane.h"
#include "cds_msgs/SignDetected.h"
#include "std_msgs/Float32.h"

#include "navigation.hpp"

using namespace sensor_msgs;

static Navigation::Lane leftLane, rightLane;
static Navigation::TurningFlags leftTurning, rightTurning;

static ros::Publisher speedPublisher, steerPublisher;

static Navigation navigation;

void convertLandMarkMsg2Lane(const std::vector<cds_msgs::LandMark>& landmarkMsg, Navigation::Lane& lane)
{
    lane.clear();
    for (size_t i = 0; i < landmarkMsg.size(); i++)
    {
        const cds_msgs::LandMark& landmark = landmarkMsg[i];
        lane.push_back(cv::Point{landmark.x, landmark.y});
    }
}

void convertTurningMsg2Turning(const std::vector<std_msgs::Bool>& turningMsg, Navigation::TurningFlags& turning)
{
    turning.clear();
    for (const std_msgs::Bool& msg : turningMsg)
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

static void laneCallback(const cds_msgs::LaneConstPtr& laneMsg)
{
    auto&& msgLeftLane = laneMsg->leftLane;
    auto&& msgRightLane = laneMsg->rightLane;
    auto&& msgLeftTurning = laneMsg->leftTurn;
    auto&& msgRightTurning = laneMsg->rightTurn;


    convertLandMarkMsg2Lane(msgLeftLane, leftLane);
    convertLandMarkMsg2Lane(msgRightLane, rightLane);
    convertTurningMsg2Turning(msgLeftTurning, leftTurning);
    convertTurningMsg2Turning(msgRightTurning, rightTurning);

    navigation.update(leftLane, rightLane);
    navigation.update(leftTurning, rightTurning);
}

static void signCallback(const cds_msgs::SignDetected& signMsg)
{
    navigation.update(signMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation");

    ros::NodeHandle nh;

    ros::Subscriber laneSub = nh.subscribe("/lane_detected", 10, laneCallback);
    ros::Subscriber signSub = nh.subscribe("/sign_detected", 10, signCallback);

    speedPublisher = nh.advertise<std_msgs::Float32>("/set_speed_car_api", 10);
    steerPublisher = nh.advertise<std_msgs::Float32>("/set_steer_car_api", 10);

    ros::Rate rate{50};

    while (ros::ok())
    {
        ros::spinOnce();

        publishSpeed(navigation.getSpeed());
        publishSteer(navigation.getSteer());
        rate.sleep();
    }

    return 0;
}