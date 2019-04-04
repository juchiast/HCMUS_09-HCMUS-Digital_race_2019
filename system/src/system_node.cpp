#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <cds_msgs/System.h>
#include <cds_msgs/SignDetected.h>
#include <string>

static ros::Publisher publisher, lcd_publisher;
static cds_msgs::System systemMsg;

bool isBtn1Pressed = false;
bool isBtn4Pressed = false;
bool isSensorDetected = false;
bool isForceStop = true;

void btn1Callback(const std_msgs::Bool& msg)
{
    // button 1 ; start button
    if (msg.data == true)
    {
        isBtn1Pressed = true;
    } else 
    {
        if (isBtn1Pressed == true)
        {
            isForceStop = false;
        }
        isBtn1Pressed = false;
    }
}
void speedCallback(const std_msgs::Float32& msg)
{
    static char buffer[255];
    float x= msg.data;
    snprintf(buffer, 255, "2:2:SPEED %.2f", x);

    std_msgs::String str_msg;
    str_msg.data = buffer;
    lcd_publisher.publish(str_msg);
}
void signCallback(const cds_msgs::SignDetected& msg)
{
    std::string x;
    if(msg.id==-1){
        x = "None ";
    }
    if(msg.id==1){
        x = "Left ";
    }
    else if(msg.id==2){
        x = "Right";
    }

    static char buffer[255];
    snprintf(buffer, 255, "2:3:SIGN %s", x.c_str());

    std_msgs::String str_msg;
    str_msg.data= buffer;
    lcd_publisher.publish(str_msg);
}

// Not work
void btn2Callback(const std_msgs::Bool& msg)
{
    // button 2
}

// Not work
void btn3Callback(const std_msgs::Bool& msg)
{
    // button 3 
}


void btn4Callback(const std_msgs::Bool& msg)
{
    // button 4 : stop button
    if (msg.data == true)
    {
        isBtn4Pressed = true;
    } else 
    {
        if (isBtn4Pressed == true)
        {
            isForceStop = true;
        }
        isBtn4Pressed = false;
    }
}

void sensorCallback(const std_msgs::Bool& msg)
{
    // sensor return false if it is activated
    isSensorDetected = !msg.data;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "system");

    ros::NodeHandle nh("~");

    std::string bt1Topic, bt2Topic, bt3Topic, bt4Topic, sensorTopic, btn2Topic, btn3Topic;
    bt1Topic = nh.param("bt1Topic", std::string("/bt1_status"));
    bt2Topic = nh.param("bt2Topic", std::string("/bt2_status"));
    bt3Topic = nh.param("bt3Topic", std::string("/bt3_status"));
    bt4Topic = nh.param("bt4Topic", std::string("/bt4_status"));
    sensorTopic = nh.param("sensorTopic", std::string("/ss_status"));

    btn2Topic = nh.param("btn2Topic", std::string("/btn2_status"));
    btn3Topic = nh.param("btn3Topic", std::string("/btn3_status"));

    nh.setParam("/lcd_adr", 0x3f);


    ROS_INFO("bt1 topic = %s", bt1Topic.c_str());
    ROS_INFO("bt2 topic = %s", bt2Topic.c_str());
    ROS_INFO("bt3 topic = %s", bt3Topic.c_str());
    ROS_INFO("bt4 topic = %s", bt4Topic.c_str());
    ROS_INFO("sensor topic = %s", sensorTopic.c_str());
    ROS_INFO("lcd_adr = %02x", 0x3f);

    ROS_INFO("btn2 topic = %s", btn2Topic.c_str());
    ROS_INFO("btn3 topic = %s", btn3Topic.c_str());

    // We dont need to use buttons now
    ros::Subscriber bt1Sub = nh.subscribe(bt1Topic, 10, btn1Callback);
    //ros::Subscriber bt2Sub = nh.subscribe(bt2Topic, 10, btn2Callback);
    //ros::Subscriber bt3Sub = nh.subscribe(bt3Topic, 10, btn3Callback);
    ros::Subscriber bt4Sub = nh.subscribe(bt4Topic, 10, btn4Callback);
    ros::Subscriber ssSub = nh.subscribe(sensorTopic, 10, sensorCallback);
    
    ros::Subscriber btn2Sub = nh.subscribe("/set_speed_car_api", 10, speedCallback);
    ros::Subscriber btn3Sub = nh.subscribe("/sign_detected", 10, signCallback);

    publisher = nh.advertise<cds_msgs::System>("/system", 10);
    lcd_publisher = nh.advertise<std_msgs::String>("/lcd_print", 10);


    systemMsg.isStop.data = true;

    ros::Rate rate{30};
    while (ros::ok())
    {
        ros::spinOnce();
        systemMsg.header.stamp = ros::Time::now();

        if (isForceStop)
        {
            systemMsg.isStop.data = true;
        } else
        {
            systemMsg.isStop.data = isSensorDetected;
        }

        // systemMsg.isStop.data = isSensorDetected;   
        // if(isBtn4Pressed || isSensorDetected)
        // {
        //     systemMsg.isStop.data = true;
        // }
        // else {
        //     systemMsg.isStop.data = false;
        // }
        publisher.publish(systemMsg);

        {
            std_msgs::String msg;
            msg.data = "6:0:HCMUS09";
            lcd_publisher.publish(msg);
        }
        rate.sleep();
    }

    return 0;
}
