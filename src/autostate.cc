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
void *autostateRecCmdHandle_thread(void *ptr);
typedef struct{
	char name;
	double x;
	double y;
	double theta;
}tag_pos;
tag_pos tagPos[100];
void readfilefstream()
{
	std::ifstream infile("/root/work_space/devel/lib/gui_node/init_pose");
	if(!infile.is_open())
	{
		printf("open /root/work_space/devel/lib/gui_node/init_pose failure \n");
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
			i++;
		}
	}
	infile.close();
}

AutoState::AutoState()
{
	pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	Goal_pub = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	Pause_pub = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);
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
	readfilefstream();
	int stateRecthread;
	if ((stateRecthread = pthread_create(&AutoStateRecCmd_thread, NULL, autostateRecCmdHandle_thread, (void*)(this))))
	{
		printf("Thread2 creation failed: %d\n", stateRecthread);
	}
	int statethread;
	if ((statethread = pthread_create(&AutoState_thread, NULL, autostateHandle_thread, (void*)(this))))
	{
		printf("Thread2 creation failed: %d\n", statethread);
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
  tf::Quaternion quat = tf::createQuaternionFromYaw(ps.a);
  goal.pose.orientation.x = quat.x();
  goal.pose.orientation.y = quat.y();
  goal.pose.orientation.z = quat.z();
  goal.pose.orientation.w = quat.w();
 // ROS_INFO("Sending Next pos");
  Goal_pub.publish(goal);
}

void AutoState::SendStop(){
	actionlib_msgs::GoalID cmd_stop;
	cmd_stop.stamp=ros::Time::now();
	cmd_stop.id="0";
//	ROS_INFO("Sending Stop pos before");
	Pause_pub.publish(cmd_stop);
//	ROS_INFO("Sending Stop pos end");
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

  for(int i = 0; i < 36; i++) initialpose.pose.covariance.elems[i]=0;
  initialpose.pose.covariance.elems[0]=0.8;
  initialpose.pose.covariance.elems[7]=0.8;
  initialpose.pose.covariance.elems[35]=0.5;

//  ROS_INFO("Sending Init pos before");
  Init_pos_pub.publish(initialpose);
//  ROS_INFO("Sending Init pos end");



}

void AutoState::setCurrobotPos(geometry_msgs::PoseWithCovarianceStamped acml_msgs)
{
	robot_pose.pose.pose.position.x 	= acml_msgs.pose.pose.position.x;
	robot_pose.pose.pose.position.y 	= acml_msgs.pose.pose.position.y;
	robot_pose.pose.pose.orientation.z 	= acml_msgs.pose.pose.orientation.z;
}


void AutoState::setCurrRobotState(int istate)
{
	goto_goalStatusFlag = istate;
}
void AutoState::setReceive(unsigned char cmd, unsigned char dest)
{
	state_cmd = cmd;
	state_dest = dest;
}

int AutoState::AcktoAndriod(unsigned char &ack,unsigned char &dest)
{
	//base time is 100ms,mian.cc call this function
	if(StateBack == MANUAL)
	{
		ack = MANUAL;
		dest = 0;
	}
	else if(StateBack == AUTO_NAVI)
	{
		switch(naviStateBack)
		{
		case GOTO_STATE:
			if(goto_goalStatus != GOTO_ACK_ARRIVE)
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
							//start timeout
							goalreach_timeoutcnt++;
							//time base is 100ms ,100 = 10s
							if((goalreach_timeoutcnt > 100) || (goto_goalStatusFlag == 1))
							{
								goto_goalStatus = GOTO_ACK_ARRIVE;
								ack = GOTO_ACK_ARRIVE;
								dest = tagPos[i].name;
							}
							else
							{
								goto_goalStatus = GOTO_ACK_PROCESS;
							}
							//
						}
						else
						{
							goalreach_timeoutcnt = 0;
							goto_goalStatus = GOTO_ACK_PROCESS;
							ack = GOTO_ACK_PROCESS;
							dest = tagPos[i].name;
						}
					}
				}
			}
			break;
		case PAUSE_STATE:
			ack = PAUSE_STATE;
			dest = 0;
			break;
		case TERMINATE_STATE:
			ack = TERMINATE_STATE;
			dest = 0;
			break;
		}
	}
	return 0;
}
void *autostateRecCmdHandle_thread(void* ptr)
{
	 AutoState *me=(AutoState*) ptr;
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
		case 0xFF://read timeout 200ms
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
				me->sendgoalFLag = false;
				me->goto_aimDelay = 0;
				me->goto_goalStatus = GOTO_ACK_PROCESS;
				me->goto_aim = me->state_dest;
				me->naviState = GOTO_STATE;
//				printf("AUTO_NAVI GotoState \n");
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
		case INITIALPOSE_STATE:
			if(me->State == AUTO_NAVI)
			{
				me->initialPose_aim = me->state_dest;
				me->naviState = INITIALPOSE_STATE;
				me->inittimes = 10;
			}
			break;
		default:
			me->State = 0;
			break;
		}
		usleep(50000);//50ms
	}
}

void *autostateHandle_thread(void* ptr)
{
	AutoState *me=(AutoState*) ptr;
	while(true)
	{
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
    			break;
    		case MANUAL_BACKWARD:
    			me->cmd_vel.linear.x  = -0.15;
    			me->cmd_vel.linear.y  = 0.0;
    			me->cmd_vel.angular.z = 0.0;
    			break;
    		case MANUAL_LEFT:
    			me->cmd_vel.linear.x  = 0.0;
    			me->cmd_vel.linear.y  = 0.0;
    			me->cmd_vel.angular.z = 0.6;
    			break;
    		case MANUAL_RIGHT:
    			me->cmd_vel.linear.x  = 0.0;
    			me->cmd_vel.linear.y  = 0.0;
    			me->cmd_vel.angular.z = -0.6;
    			break;
    		case MANUAL_STOP:
    			me->cmd_vel.linear.x  = 0.0;
    			me->cmd_vel.linear.y  = 0.0;
    			me->cmd_vel.angular.z = 0.0;
    			break;
    		default:
    			me->cmd_vel.linear.x  = 0.0;
    			me->cmd_vel.linear.y  = 0.0;
    			me->cmd_vel.angular.z = 0.0;
    			me->manualState = MANUAL_STOP;
    			break;
    		}
    		me->pub_.publish(me->cmd_vel);
    		me->StateBack = MANUAL;
    		break;
    	case AUTO_NAVI:
    		switch(me->naviState)
    		{
    		case GOTO_STATE:
    			if((me->goto_aim != me->goto_aimBak) && (me->naviStateBack == PAUSE_STATE))
    			{
    				me->naviState = PAUSE_STATE;//
    			}
    			else
    			{
//    				printf("Auto Navi GotoState Send \n");
    				me->naviStateBack = GOTO_STATE;
    				if(me->goto_goalStatus != GOTO_ACK_ARRIVE)
    				{
        				for(int i = 0; i < 10; i++)
        				{
        					if(me->goto_aim == tagPos[i].name)
        					{
        						Pos pos;
        						pos.x = tagPos[i].x;
        						pos.y = tagPos[i].y;
        						pos.a = tagPos[i].theta;
        						me->SendNextPos(pos);

        						me->sendgoalFLag = true;
        						me->goto_aimBak = me->goto_aim;

        					}
        				}
    				}
    			}
    			break;
    		case PAUSE_STATE:
    			me->SendStop();
    			me->naviStateBack = PAUSE_STATE;
    			break;
    		case TERMINATE_STATE:
    			me->SendStop();
    			me->naviStateBack = TERMINATE_STATE;
    			break;
    		case INITIALPOSE_STATE:
    			if(me->inittimes > 0)
    			{
    				for(int i = 0; i < 10; i++)
					{
						if(me->initialPose_aim == tagPos[i].name)
						{
							me->SendInitPos(tagPos[i].x,tagPos[i].y,tagPos[i].theta);
							me->initialPose_aimBak = me->initialPose_aim;
							break;
						}
					}
    				me->inittimes--;
    				if(me->inittimes <= 0)
    				{
    					me->inittimes = 0;
    				}
    			}
    			me->naviStateBack = INITIALPOSE_STATE;
    			break;
    		default:
    			me->naviState = TERMINATE_STATE;
    			break;
    		}
    		sleep(1);//1s
    		me->StateBack = AUTO_NAVI;
    		break;
    	default:
    		ROS_INFO("Robot default!");
    		break;
    	}
    	usleep(50000);//50ms
    }
}




