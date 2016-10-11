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

//void readfile()
//{
//
//    FILE *fp1;
//    int i;
//    struct student
//    {
//        char name[1];
//        double x;
//        double y;
//        double theta;
//    }stu;
//    if((fp1=fopen("test2.txt","rb"))==NULL)
//    {
//        printf("");
//        exit(0);
//    }
//    printf(":\n");
//    for (i = 0; i < 3; i++)
//    {
//        fread(&stu,sizeof(stu),1,fp1);
//        printf("%s %f %f %f\n",stu.name, stu.x, stu.y,stu.theta);
//    }
//    fclose(fp1);
//}
AutoState  *myState;
geometry_msgs::Twist cmd_vel;
ros::Publisher move_pub;
void updatapose(const geometry_msgs::PoseWithCovarianceStamped& acml_msgs)
{
	double x,y,z;
//	robot_pose.pose.pose.position.x=pose.pose.pose.position.x;
//	robot_pose.pose.pose.position.y=pose.pose.pose.position.y;
//	robot_pose.pose.pose.orientation.z=pose.pose.pose.orientation.z;
	myState->setCurrobotPos(acml_msgs);
}
void shutdown(int sig) {
  ROS_INFO("[GUI_Node] disconnect gui node ...");
  ros::shutdown();
}
char TestBuf[10][2]=
{
MANUAL,0,
MANUAL_FORWARD,0,
MANUAL_BACKWARD,0,
MANUAL_LEFT,0,
MANUAL_RIGHT,0,
MANUAL_STOP,0,
AUTO_NAVI,0,
GOTO_STATE,'A',
PAUSE_STATE,0,
TERMINATE_STATE,0,
};
int testCnt = 0;
int main(int argc, char** argv) {

  ros::init(argc, argv, "gui_node");
  signal(SIGINT, shutdown);

  ros::NodeHandle nh;
  myState = new AutoState;
  SerialPort mySerial;
  ros::Subscriber Pose_sub = nh.subscribe("amcl_pose",10,updatapose);
  ROS_INFO("[GUI_Node] launch ...");
  sleep(2);
  ros::Rate loop_rate(5);
  while (ros::ok()) {
	unsigned char ackStatus,ackDest;

//	mySerial.data0 = TestBuf[testCnt][0];
//	mySerial.data1 = TestBuf[testCnt][1];
	myState->setReceive(mySerial.data0, mySerial.data1);
	ROS_INFO("[REC_Data]:%d,%d",mySerial.data0,mySerial.data1);

	testCnt++;
	if(testCnt >= 10)
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




