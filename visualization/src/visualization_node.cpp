#include <ros/ros.h>

#include "cds_msgs/SignDetected.h"
#include "cds_msgs/Lane.h"

#include <opencv2/highgui.hpp>

cds_msgs::SignDetected lastSignMsg;

static void signMsgCallback(const cds_msgs::SignDetected &msg)
{
    lastSignMsg = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualization");

    ros::spin();

    return 0;
}