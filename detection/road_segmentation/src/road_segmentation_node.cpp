#include <ros/ros.h>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>

#include <sensor_msgs/Image.h>

#include "road_segmentation.hpp"

static RoadSegmentation segmenter;
static image_transport::Publisher publisher;
static cv_bridge::CvImagePtr bridgePublish(new cv_bridge::CvImage);


static void publish()
{
    cv::Mat binary = segmenter.getSegmentation();
    sensor_msgs::Image msg;

    ros::Time time = ros::Time::now();
    bridgePublish->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    bridgePublish->header.stamp = time;
    bridgePublish->header.frame_id = "/road_segementation";

    bridgePublish->image = binary;
    publisher.publish(bridgePublish->toImageMsg());
}

static void visualize()
{
    cv::Mat binary = segmenter.getSegmentation();
    cv::imshow("Segmentation", binary);
    cv::waitKey(1);
}

static void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        int key = cv::waitKey(1);
        if (key == 'c' || key == 'C') {
            const std::string time = std::to_string(ros::Time::now().toNSec());
            const std::string name = "./capture_" + time + ".jpg";
            bool flag = cv::imwrite(name, cv_ptr->image);
            if (flag) {
                ROS_INFO("Capture: %s", name.c_str());
            }
        }
        segmenter.detect(cv_ptr->image);
        visualize();
        publish();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "road_segmentation");

    ros::NodeHandle nh;
    std::string sub_camera_topic = nh.param("sub_camera_topic", std::string("/camera/rgb/image_raw"));
    std::string pub_binary_topic = nh.param("pub_road_topic", std::string("/road_segmentation/image_raw"));

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(sub_camera_topic, 10, imageCallback);
    
    publisher = it.advertise(pub_binary_topic, 1);

    ROS_INFO("Started road_segmentation");

    ros::spin();

    cv::destroyAllWindows();

    return 0;
}