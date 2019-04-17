#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
 
void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  float pos[22],vel[22];
 // pos=msg.position;
  pos[0]=msg->position[0];
  pos[1]=msg->position[1];
  pos[2]=msg->position[2];
  pos[3]=msg->position[3];
  pos[4]=msg->position[4];
  pos[5]=msg->position[5];
  pos[6]=msg->position[6];
  pos[7]=msg->position[7];
  pos[8]=msg->position[8];
  pos[9]=msg->position[9];
  pos[10]=msg->position[10];
  pos[11]=msg->position[11];
  pos[12]=msg->position[12];
  pos[13]=msg->position[13];
  pos[14]=msg->position[14];
  pos[15]=msg->position[15];
  pos[16]=msg->position[16];
  pos[17]=msg->position[17];
  pos[18]=msg->position[18];
  pos[19]=msg->position[19];
  pos[20]=msg->position[20];
  pos[21]=msg->position[21];

  vel[0]=msg->velocity[0];
  vel[1]=msg->velocity[1];
  vel[2]=msg->velocity[2];
  vel[3]=msg->velocity[3];
  vel[4]=msg->velocity[4];
  vel[5]=msg->velocity[5];
  vel[6]=msg->velocity[6];
  vel[7]=msg->velocity[7];
  vel[8]=msg->velocity[8];
  vel[9]=msg->velocity[9];
  vel[10]=msg->velocity[10];
  vel[11]=msg->velocity[11];
  vel[12]=msg->velocity[12];
  vel[13]=msg->velocity[13];
  vel[14]=msg->velocity[14];
  vel[15]=msg->velocity[15];
  vel[16]=msg->velocity[16];
  vel[17]=msg->velocity[17];
  vel[18]=msg->velocity[18];
  vel[19]=msg->velocity[19];
  vel[20]=msg->velocity[20];
  vel[21]=msg->velocity[21];

  ROS_INFO("I heard: [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f] [%f]",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6],pos[7],pos[8],pos[9],pos[10],pos[11],pos[12],pos[13],pos[14],pos[15],pos[16],pos[17],pos[18],pos[19],pos[20],pos[21],vel[0],vel[1],vel[2],vel[3],vel[4],vel[5],vel[6],vel[7],vel[8],vel[9],vel[10],vel[11],vel[12],vel[13],vel[14],vel[15],vel[16],vel[17],vel[18],vel[19],vel[20],vel[21]);
    std::cout<<"jointstatesCallback run"<<std::endl;
 
}
 
int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener");
 
 
  ros::NodeHandle n;
 
 
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, jointstatesCallback);
 
  std::cout<<"successful"<<std::endl;
 
  ros::spin();
 
 
  return 0;
}
