#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cds_msgs/System.h>

static bool btn1Pressed = false;
static bool btn2Pressed = false;
static bool btn3Pressed = false;
static bool btn4Pressed = false;
static bool distanceSensor = false;

void btn1Callback(const std_msgs::Bool& msg)
{
    if (msg.data)
    {
        btn1Pressed = true;
    }
    else
    {
        if (btn1Pressed)
        {
            ROS_INFO("btn1 released");
            // TODO: do somethings when user release btn1
        }
        btn1Pressed = false;
    } 
}

void btn2Callback(const std_msgs::Bool& msg)
{
    if (msg.data)
    {
        btn2Pressed = true;
    }
    else
    {
        if (btn2Pressed)
        {
            ROS_INFO("btn2 released");
            // TODO: do somethings when user release btn2
        }
        btn2Pressed = false;
    } 
}


void btn3Callback(const std_msgs::Bool& msg)
{
    if (msg.data)
    {
        btn3Pressed = true;
    }
    else
    {
        if (btn3Pressed)
        {
            ROS_INFO("btn3 released");
            // TODO: do somethings when user release btn3
        }
        btn3Pressed = false;
    } 
}


void btn4Callback(const std_msgs::Bool& msg)
{
    if (msg.data)
    {
        btn4Pressed = true;
    } 
    else
    {
        if (btn4Pressed)
        {
            ROS_INFO("btn4 released");
            // TODO: do somethings when user release btn4
        }
        btn4Pressed = false;
    } 
}

void sensorCallback(const std_msgs::Bool& msg)
{
    if (msg.data != distanceSensor)
    {
        distanceSensor = msg.data;
        
        cds_msgs::System systemMsg;
        systemMsg.header.stamp = ros::Time::now();
        systemMsg.isStop.data = true;
    }
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

    nh.subscribe(bt1Topic, 10, btn1Callback);
    nh.subscribe(bt2Topic, 10, btn2Callback);
    nh.subscribe(bt3Topic, 10, btn3Callback);
    nh.subscribe(bt4Topic, 10, btn4Callback);
    nh.subscribe(sensorTopic, 10, sensorCallback);

    ros::spin();


    return 0;
}