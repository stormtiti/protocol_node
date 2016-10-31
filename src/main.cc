/*
 * main.cc
 *
 *  Created on: Oct 8, 2016
 *      Author: root
 */

// ros library usage
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <signal.h>
#include <tf/message_filter.h>
// c++ std lib
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>     /*标准函数库定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include "autostate.h"
#include "serialport.h"
#include <fstream>
#include <std_msgs/Int16.h>
AutoState  *myState;
geometry_msgs::Twist cmd_vel;
ros::Publisher move_pub;
void updatapose(const geometry_msgs::PoseWithCovarianceStamped& acml_msgs)
{
	double x,y,z;
	myState->setCurrobotPos(acml_msgs);
}

void goalRead_fun(const std_msgs::Int16 &goalreached)
{
	double x,y,z;
	printf("goalreaded flag is:%d \n",goalreached.data);
	myState->setCurrRobotState(goalreached.data);
}

void shutdown(int sig) {
  ROS_INFO("[GUI_Node] disconnect gui node ...");
  ros::shutdown();
}
//char TestBuf[12][2]=
//{
//MANUAL,0,
//MANUAL_FORWARD,0,
//MANUAL_BACKWARD,0,
//MANUAL_LEFT,0,
//MANUAL_RIGHT,0,
//MANUAL_STOP,0,
//AUTO_NAVI,0,
//GOTO_STATE,'B',
//PAUSE_STATE,0,
//GOTO_STATE,'B',
//TERMINATE_STATE,0,
//INITIALPOSE_STATE,'C'
//};
char TestBuf[12][2]=
{
AUTO_NAVI,0,
GOTO_STATE,'B',
0xFF,0,
0xFF,0,
0xFF,0,
0xFF,0,
PAUSE_STATE,0,
0xFF,0,
0xFF,0,
GOTO_STATE,'C',
0xFF,0,
0xFF,0
};
int testCnt = 0;
int main(int argc, char** argv) {

  ros::init(argc, argv, "gui_node");
  signal(SIGINT, shutdown);
  static  char recback;
  ros::NodeHandle nh;
  myState = new AutoState;
  SerialPort mySerial;
  ros::Subscriber Pose_sub = nh.subscribe("amcl_pose",10,updatapose);
  ros::Subscriber goalReach_sub = nh.subscribe("goal_reached",10,goalRead_fun);
  ROS_INFO("[GUI_Node] launch ...");
  sleep(2);
  ros::Rate loop_rate(10);
  while (ros::ok()) {
	unsigned char ackStatus,ackDest;

//	mySerial.data0 = TestBuf[testCnt][0];
//	mySerial.data1 = TestBuf[testCnt][1];

	myState->setReceive(mySerial.data0, mySerial.data1);
	if(recback != mySerial.data0)
	{
//		 ROS_INFO("[REC_Data]:%d,%d",mySerial.data0,mySerial.data1);
		 recback = mySerial.data0;
	}


	testCnt++;
	if(testCnt >= 12)
	{
		testCnt = 0;
	}
	unsigned char buf[2];
	if(!myState->AcktoAndriod(ackStatus,ackDest))
	{
		buf[0] = ackStatus;
		buf[1] = ackDest;
		mySerial.writeData(&buf[0],2);
	}
    ros::spinOnce();
    loop_rate.sleep();
  } // End of ros::ok
  return 0;
}




