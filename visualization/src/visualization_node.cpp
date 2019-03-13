#include <ros/ros.h>
#include "cds_msgs/SignDetectedArray.h"

#include <opencv2/highgui.hpp>

static void signDetectedVisualization(const cds_msgs::SignDetectedArray& msg)
{

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualization");

    ros::spin();

    return 0;
}