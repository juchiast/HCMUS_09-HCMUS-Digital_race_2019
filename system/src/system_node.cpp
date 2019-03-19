#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cds_msgs/System.h>

static ros::Publisher publisher;
static cds_msgs::System systemMsg;

void btn1Callback(const std_msgs::Bool& msg)
{
    // button 1
}

void btn2Callback(const std_msgs::Bool& msg)
{
    // button 2
}


void btn3Callback(const std_msgs::Bool& msg)
{
    // button 3 
}


void btn4Callback(const std_msgs::Bool& msg)
{
    // button 4 
}

void sensorCallback(const std_msgs::Bool& msg)
{
    // sensor return false if it is activated
    systemMsg.isStop.data = !msg.data; 
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "system");

    ros::NodeHandle nh("~");

    std::string bt1Topic, bt2Topic, bt3Topic, bt4Topic, sensorTopic;
    bt1Topic = nh.param("bt1Topic", std::string("/bt1_status"));
    bt2Topic = nh.param("bt2Topic", std::string("/bt2_status"));
    bt3Topic = nh.param("bt3Topic", std::string("/bt3_status"));
    bt4Topic = nh.param("bt4Topic", std::string("/bt4_status"));
    sensorTopic = nh.param("sensorTopic", std::string("/ss_status"));

    ROS_INFO("bt1 topic = %s", bt1Topic.c_str());
    ROS_INFO("bt2 topic = %s", bt2Topic.c_str());
    ROS_INFO("bt3 topic = %s", bt3Topic.c_str());
    ROS_INFO("bt4 topic = %s", bt4Topic.c_str());
    ROS_INFO("sensor topic = %s", sensorTopic.c_str());

    // We dont need to use buttons now
    ros::Subscriber bt1Sub = nh.subscribe(bt1Topic, 10, btn1Callback);
    ros::Subscriber bt2Sub = nh.subscribe(bt2Topic, 10, btn2Callback);
    ros::Subscriber bt3Sub = nh.subscribe(bt3Topic, 10, btn3Callback);
    ros::Subscriber bt4Sub = nh.subscribe(bt4Topic, 10, btn4Callback);
    ros::Subscriber ssSub = nh.subscribe(sensorTopic, 10, sensorCallback);

    publisher = nh.advertise<cds_msgs::System>("/system", 10);

    ros::Rate rate{30};
    while (ros::ok())
    {
        ros::spinOnce();
        systemMsg.header.stamp = ros::Time::now();

        publisher.publish(systemMsg);
        rate.sleep();
    }


    return 0;
}