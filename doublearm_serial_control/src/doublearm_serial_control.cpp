#include "ros/ros.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "sensor_msgs/JointState.h"
#include <string>
#include <sstream>
serial::Serial ros_ser;       //声明串口对象
//回调函数
void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
	std::stringstream str;
	str<<"right_joint2: pos:"<<msg->position[0]<<" vel:"<<msg->velocity[0]
	   <<"left_joint2: pos:"<<msg->position[1]<<" vel:"<<msg->velocity[1]
	   <<"right_joint3: pos:"<<msg->position[2]<<" vel:"<<msg->velocity[2]
       <<"left_joint3: pos:"<<msg->position[3]<<" vel:"<<msg->velocity[3]
	   <<"right_joint4: pos:"<<msg->position[4]<<" vel:"<<msg->velocity[4]
	   <<"left_joint4: pos:"<<msg->position[5]<<" vel:"<<msg->velocity[5]
       <<"right_joint5: pos:"<<msg->position[6]<<" vel:"<<msg->velocity[6]
	   <<"left_joint5: pos:"<<msg->position[7]<<" vel:"<<msg->velocity[7]
	   <<"right_joint6: pos:"<<msg->position[8]<<" vel:"<<msg->velocity[8]
       <<"left_joint6: pos:"<<msg->position[9]<<" vel:"<<msg->velocity[9]
	   <<"left_palm_joint: pos:"<<msg->position[10]<<" vel:"<<msg->velocity[10]
	   <<"right_palm_joint: pos:"<<msg->position[11]<<" vel:"<<msg->velocity[11]
       <<"left_thumb_joint: pos:"<<msg->position[12]<<" vel:"<<msg->velocity[12]
	   <<"right_thumb_joint: pos:"<<msg->position[13]<<" vel:"<<msg->velocity[13]
	   <<"left_index_finger_joint: pos:"<<msg->position[14]<<" vel:"<<msg->velocity[14]
       <<"right_index_finger_joint:"<<msg->position[15]<<" vel:"<<msg->velocity[15]
	   <<"left_middle_finger_joint: pos:"<<msg->position[16]<<" vel:"<<msg->velocity[16]
	   <<"right_middle_finger_joint: pos:"<<msg->position[17]<<" vel:"<<msg->velocity[17]
       <<"left_third_finger_joint: pos:"<<msg->position[18]<<" vel:"<<msg->velocity[18]
	   <<"right_third_finger_joint: pos:"<<msg->position[19]<<" vel:"<<msg->velocity[19]
	   <<"left_little_finger_joint: pos:"<<msg->position[20]<<" vel:"<<msg->velocity[20]
       <<"right_little_finger_joint: pos:"<<msg->position[21]<<" vel:"<<msg->velocity[21]<<std::endl;
     ROS_INFO_STREAM("Writing to serial port" );
     ros_ser.write(str.str());         //发送串口数据
}
 
/*void callback(const std_msgs::String::ConstPtr& msg)
{
	std::stringstream str;
	str<<"joint1: pos:"<<1.0<<" vel:"<<2.0<<std::endl;
     ROS_INFO_STREAM("Write to serial port" << msg->data);
     ros_ser.write(str.str());         //发送串口数据
 }*/
int main (int argc, char** argv)
{
     ros::init(argc, argv, "doublearm_serial_control");  //初始化节点
     ros::NodeHandle n;                        //声明节点句柄
     //订阅主题/joint_states，并配置回调函数
     ros::Subscriber command_sub = n.subscribe("/joint_states", 1000, jointstatesCallback);
     //发布主题sensor
     ros::Publisher sensor_pub = n.advertise<std_msgs::String>("sensor", 1000);
 
     try
     {
         //设置串口属性，并打开串口
         ros_ser.setPort("/dev/ttyUSB0");
         ros_ser.setBaudrate(9600);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_ser.setTimeout(to);
         ros_ser.open();
     }
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
     }
 
     //检测串口是否已经打开，并给出提示信息 
     if(ros_ser.isOpen()){
         ROS_INFO_STREAM("Serial Port opened");
     }
     else
        {
         return -1;
        }
 
     //指定循环的频率
     ros::Rate loop_rate(10);
 
     while(ros::ok()){
 
         //处理ROS的信息，比如订阅消息,并调用回调函数
         ros::spinOnce();
 
         if(ros_ser.available())
         {
             ROS_INFO_STREAM("Reading from serial port");
             std_msgs::String serial_data;
             //获取串口数据
             serial_data.data = ros_ser.read(ros_ser.available());
             ROS_INFO_STREAM("Read: " << serial_data.data);
             //将串口数据发布到主题sensor
             sensor_pub.publish(serial_data);
         }
         loop_rate.sleep();
     }
 }
