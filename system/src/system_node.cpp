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

    // We dont need to use buttons now
    // nh.subscribe(bt1Topic, 10, btn1Callback);
    // nh.subscribe(bt2Topic, 10, btn2Callback);
    // nh.subscribe(bt3Topic, 10, btn3Callback);
    // nh.subscribe(bt4Topic, 10, btn4Callback);
    nh.subscribe(sensorTopic, 10, sensorCallback);

    publisher = nh.advertise<cds_msgs::System>("/system", 10);

    ros::Rate rate{30};
    while (ros::ok())
    {
        systemMsg.header.stamp = ros::Time::now();

        publisher.publish(systemMsg);
        rate.sleep();
    }


    return 0;
}