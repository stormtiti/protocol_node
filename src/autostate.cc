/*
 * main.cc
 *
 *  Created on: Oct 8, 2016
 *      Author: root
 */

// ros library usage
#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <tf/message_filter.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// c++ std lib
#include <fstream>
#include <map>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>     /*标准函数库定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include "autostate.h"

#define BUFFER_LENGTH 1024  //Max length of buffer
using namespace std;
#define FALSE  -1
#define TRUE   0
void *autostateHandle_thread(void* ptr);
typedef struct{
	char name;
	double x;
	double y;
	double theta;
}tag_pos;
tag_pos tagPos[100];
void readfilefstream()
{
	std::ifstream infile("init_pose");
	if(!infile.is_open())
	{
		printf("open failure \n");
	}
	else
	{
		std::string line;
		int i = 0;
		while(std::getline(infile,line))
		{
			std::istringstream iss(line);
			char name;
			double x,y,theta;
			iss >> name >> x >> y >> theta;
			tagPos[i].name = name;
			tagPos[i].x = x;
			tagPos[i].y = y;
			tagPos[i].theta = theta;
			printf("%d,%f,%f,%f \n",name,x,y,theta);
			i++;
		}
	}
	infile.close();
}

AutoState::AutoState()
{
	  pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	  Goal_pub = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel",10);
	  Init_pos_pub = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",10);

	  State = MANUAL;
	  StateBack = MANUAL;
	  manualState = MANUAL_STOP;
	  naviState = TERMINATE_STATE;
	  naviStateBack = TERMINATE_STATE;
	  goto_aim = 0;
	  goto_aimBak = 0;
	  state_cmd = MANUAL;
	  state_dest = 0;
	int statethread;
	if ((statethread = pthread_create(&AutoState_thread, NULL, autostateHandle_thread, (void*)(this))))
	{
		printf("Thread2 creation failed: %d\n", statethread);
	}
	readfilefstream();
	ROS_INFO("[initialpose]%f,%f\n",tagPos[0].x,tagPos[0].y);
	for(int i = 0; i < 10 ; i++)
	{
		SendInitPos(tagPos[0].x,tagPos[0].y,tagPos[0].theta);
		usleep(10000);// 10ms
	}

}

AutoState::~AutoState()
{

}
void AutoState::SendNextPos(Pos ps){
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.seq = 0;
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = ps.x;
  goal.pose.position.y = ps.y;
  goal.pose.position.z = 0;
  goal.pose.orientation.x = 0;
  goal.pose.orientation.y = 0;
  goal.pose.orientation.z = ps.a;
  goal.pose.orientation.w = sqrt(1-ps.a*ps.a);
  ROS_INFO("Sending Next pos");

  Goal_pub.publish(goal);

//  b_destination = false;
}

void AutoState::SendStop(){
	actionlib_msgs::GoalID cmd_stop;
	cmd_stop.stamp=ros::Time::now();
	cmd_stop.id="0";
	Pause_pub.publish(cmd_stop);
}


void AutoState::SendInitPos(float x, float y,float a){
  geometry_msgs::PoseWithCovarianceStamped initialpose;

  initialpose.header.stamp = ros::Time::now();
  initialpose.header.frame_id = "map";

  initialpose.pose.pose.position.x = x;
  initialpose.pose.pose.position.y = y;
  initialpose.pose.pose.position.z = 0;

  tf::Quaternion quat = tf::createQuaternionFromYaw(a);
  initialpose.pose.pose.orientation.x = quat.x();
  initialpose.pose.pose.orientation.y = quat.y();
  initialpose.pose.pose.orientation.z = quat.z();
  initialpose.pose.pose.orientation.w = quat.w();


//  initialpose.pose.pose.orientation.x = 0;
//  initialpose.pose.pose.orientation.y = 0;
//  initialpose.pose.pose.orientation.z = a;
//  initialpose.pose.pose.orientation.w =sqrt(1-a*a);

  int i;
  for(i=0;i<36;i++) initialpose.pose.covariance.elems[i]=0;
  initialpose.pose.covariance.elems[0]=0.8;
  initialpose.pose.covariance.elems[7]=0.8;
  initialpose.pose.covariance.elems[35]=0.5;

//  robot_pose.pose.pose.position.x=x;
//  robot_pose.pose.pose.position.y=y;
////  robot_pose.pose.pose.orientation.z=a;
////  robot_pose.pose.pose.orientation.w=sqrt(1-a*a);
//  robot_pose.pose.pose.orientation = tf::createQuaternionFromYaw(a);

  ROS_INFO("Sending Init pos before");
  ROS_INFO("[initialpose]%f,%f\n",
		  initialpose.pose.pose.position.x,initialpose.pose.pose.position.y);
  Init_pos_pub.publish(initialpose);
  ROS_INFO("Sending Init pos end");

}

void AutoState::setCurrobotPos(geometry_msgs::PoseWithCovarianceStamped acml_msgs)
{
	robot_pose.pose.pose.position.x 	= acml_msgs.pose.pose.position.x;
	robot_pose.pose.pose.position.y 	= acml_msgs.pose.pose.position.y;
	robot_pose.pose.pose.orientation.z 	= acml_msgs.pose.pose.orientation.z;
}

void AutoState::setReceive(unsigned char cmd, unsigned char dest)
{
	state_cmd = cmd;
	state_dest = dest;
}

int AutoState::AcktoAndriod(unsigned char &ack,unsigned char &dest)
{
	if((State == AUTO_NAVI) && (naviState == GOTO_STATE))
	{
		for(int i = 0; i < 10; i++)
		{
			if(goto_aim == tagPos[i].name)
			{
				Pos pos;
				pos.x = tagPos[i].x;
				pos.y = tagPos[i].y;
				pos.a = tagPos[i].theta;
				if((abs(robot_pose.pose.pose.position.x - pos.x) < 0.3)
					&& (abs(robot_pose.pose.pose.position.y - pos.y) < 0.3))
				{
					ack = navAck;
					ack = GOTO_ACK_ARRIVE;
					dest = tagPos[i].name;
				}
				else
				{
					ack = navAck;
					ack = GOTO_ACK_PROCESS;
					dest = tagPos[i].name;
				}
			}
		}
		return 0;
	}
	else
	{
		ack = 0;
		return -1;
	}
}

void *autostateHandle_thread(void* ptr)
{
    AutoState *me=(AutoState*) ptr;
    printf("autostateHandle_thread \n");
    while(true)
    {
		///////////////receive from android//////////////////
		switch(me->state_cmd)
		{
		case MANUAL:
			me->State = MANUAL;
			break;
		case MANUAL_FORWARD:
			if(me->State == MANUAL)
			{
				me->manualState = MANUAL_FORWARD;
			}
			break;
		case MANUAL_BACKWARD:
			if(me->State == MANUAL)
			{
				me->manualState = MANUAL_BACKWARD;
			}
			break;
		case MANUAL_LEFT:
			if(me->State == MANUAL)
			{
				me->manualState = MANUAL_LEFT;
			}
			break;
		case MANUAL_RIGHT:
			if(me->State == MANUAL)
			{
				me->manualState = MANUAL_RIGHT;
			}
			break;
		case MANUAL_STOP:
			if(me->State == MANUAL)
			{
				me->manualState = MANUAL_STOP;
			}
			break;
		case AUTO_NAVI:
			me->State = AUTO_NAVI;
			break;
		case GOTO_STATE:
			if(me->State == AUTO_NAVI)
			{
				me->naviState = GOTO_STATE;
			}
			break;
		case PAUSE_STATE:
			if(me->State == AUTO_NAVI)
			{
				me->naviState = PAUSE_STATE;
			}
			break;
		case TERMINATE_STATE:
			if(me->State == AUTO_NAVI)
			{
				me->naviState = TERMINATE_STATE;
			}
			break;
		default:
			me->State = 0;
			break;
		}
    	//handle
    	switch(me->State)
    	{
    	case MANUAL:
    		switch(me->manualState)
    		{
    		case MANUAL_FORWARD:
    			me->cmd_vel.linear.x  = 0.3;
    			me->cmd_vel.linear.y  = 0.0;
    			me->cmd_vel.angular.z = 0.0;
    			ROS_INFO("Robot Forward Move !");
    			break;
    		case MANUAL_BACKWARD:
    			me->cmd_vel.linear.x  = -0.3;
    			me->cmd_vel.linear.y  = 0.0;
    			me->cmd_vel.angular.z = 0.0;
    			ROS_INFO("Robot Backward Move !");
    			break;
    		case MANUAL_LEFT:
    			me->cmd_vel.linear.x  = 0.0;
    			me->cmd_vel.linear.y  = 0.0;
    			me->cmd_vel.angular.z = 0.6;
    			ROS_INFO("Robot Turn Left");
    			break;
    		case MANUAL_RIGHT:
    			me->cmd_vel.linear.x  = 0.0;
    			me->cmd_vel.linear.y  = 0.0;
    			me->cmd_vel.angular.z = -0.6;
    			ROS_INFO("Robot Turn Right");
    			break;
    		case MANUAL_STOP:
    			me->cmd_vel.linear.x  = 0.0;
    			me->cmd_vel.linear.y  = 0.0;
    			me->cmd_vel.angular.z = 0.0;
    			ROS_INFO("Robot MANUAL_STOP");
    			break;
    		default:
    			me->cmd_vel.linear.x  = 0.0;
    			me->cmd_vel.linear.y  = 0.0;
    			me->cmd_vel.angular.z = 0.0;
    			me->manualState = MANUAL_STOP;
    			ROS_INFO("Robot MANUAL_default");
    			break;
    		}
    		me->pub_.publish(me->cmd_vel);
    		me->StateBack = MANUAL;
    		break;
    	case AUTO_NAVI:
    		ROS_INFO("Robot AUTO_NAVI Move !");
    		switch(me->naviState)
    		{
    		case GOTO_STATE:
    			if(me->naviStateBack == PAUSE_STATE)
    			{
    				if(me->goto_aim != me->goto_aimBak)
    				{
    					//
    				}
    			}
    			else
    			{
    				ROS_INFO("Robot GOTO_STATE Move ! %d",me->goto_aim);
    				me->goto_aim = me->state_dest;
    				me->naviStateBack = GOTO_STATE;
    				for(int i = 0; i < 10; i++)
    				{
    					if(me->goto_aim == tagPos[i].name)
    					{
    						Pos pos;
    						pos.x = tagPos[i].x;
    						pos.y = tagPos[i].y;
    						pos.a = tagPos[i].theta;
    						me->SendNextPos(pos);
    					}
    				}
    			}
    			break;
    		case PAUSE_STATE:
    			ROS_INFO("Robot PAUSE_STATE!");
    			me->goto_aimBak = me->goto_aim;
    			me->naviStateBack = PAUSE_STATE;
    			break;
    		case TERMINATE_STATE:
    			ROS_INFO("Robot TERMINATE_STATE!");
    			me->naviStateBack = TERMINATE_STATE;
    			break;
    		default:
    			ROS_INFO("Robot AUTO_NAVI default!");
    			me->naviState = TERMINATE_STATE;
    			break;
    		}
    		me->StateBack = AUTO_NAVI;
    		break;
    	default:
    		ROS_INFO("Robot default!");
    		break;
    	}
    	usleep(50000);//50ms
    }
}


///////////////get topic from navi//////////////////
//switch()
//{
//
//}




