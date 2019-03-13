#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "cds_msgs/Lane.h"
#include "cds_msgs/SignDetectedArray.h"
#include "std_msgs/Float32.h"

#include <opencv2/opencv.hpp>

#include "navigation.hpp"

static std::vector<cv::Point> leftLane, rightLane;
static std::vector<cds_msgs::SignDetected> signs;

static ros::Publisher speedPublisher, steerPublisher;

static Navigation navigation;

static void perceptionCallback(const cds_msgs::LaneConstPtr& laneMsg, const cds_msgs::SignDetectedArrayConstPtr& signMsg);

void convertLandMarkMsg2Lane(const std::vector<cds_msgs::LandMark>& landmarkMsg, std::vector<cv::Point>& lane)
{
    lane.clear();
    for (size_t i = 0; i < landmarkMsg.size(); i++)
    {
        const cds_msgs::LandMark& landmark = landmarkMsg[i];
        lane.push_back(cv::Point{landmark.x, landmark.y});
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

static void perceptionCallback(const cds_msgs::LaneConstPtr& laneMsg, const cds_msgs::SignDetectedArrayConstPtr& signMsg)
{
    auto&& msgLeftLane = laneMsg->leftLane;
    auto&& msgRightLane = laneMsg->rightLane;

    convertLandMarkMsg2Lane(msgLeftLane, leftLane);
    convertLandMarkMsg2Lane(msgRightLane, rightLane);

    signs = signMsg->data;

    navigation.update(leftLane, rightLane, signs);

    publishSpeed(navigation.getSpeed());
    publishSteer(navigation.getSteer());
}

int main(int argc, char** argv)
{
    using namespace ros;
    using namespace message_filters;
    using namespace cds_msgs;

    ros::init(argc, argv, "navigation");

    ros::NodeHandle nodeHandle;

    message_filters::Subscriber<cds_msgs::Lane> laneSub(nodeHandle, "/lane_detected", 1);
    message_filters::Subscriber<cds_msgs::SignDetectedArray> signSub(nodeHandle, "/sign_detected", 1);

    typedef sync_policies::ApproximateTime<Lane, SignDetectedArray> MySyncPolicy;

    // // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer sync(MySyncPolicy(10), laneSub, signSub);
    sync.registerCallback(boost::bind(&perceptionCallback, _1, _2));

    // message_filters::Synchronizer<MySyncPolicy> sync{MySyncPolicy{10}, laneSub, signSub};
    // sync.registerCallback(std::bind(&perceptionCallback, _1, _2));
    // sync.registerCallback(&perceptionCallback);

    // ros::Subscriber subLane = nodeHandle.subscribe("/lane_detected", 10, laneCallback);
    // ros::Subscriber subSign = nodeHandle.subscribe("/sign_detected", 10, laneCallback);

    speedPublisher = nodeHandle.advertise<std_msgs::Float32>("/set_speed_car_api", 10);
    steerPublisher = nodeHandle.advertise<std_msgs::Float32>("/set_steer_car_api", 10);

    ros::spin();

    return 0;
}