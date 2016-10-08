/*
 * main.cc
 *
 *  Created on: Oct 8, 2016
 *      Author: root
 */

// ros library usage
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

// c++ std lib
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>

#include <sys/poll.h>
#include <termios.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <signal.h>

#define BUFFER_LENGTH 512  //Max length of buffer
#define PORT 8005

struct sockaddr_in si;
int desc;
ros::Publisher move_pub;
geometry_msgs::Twist cmd_vel;

void shutdown(int sig) {
  ROS_INFO("[GUI_Node] disconnect gui node ...");
  ros::shutdown();
}

void init_socket() {

  if ((desc =socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    ROS_ERROR("[GUI_Node] socket die");
  }

  memset((char *) &si, 0, sizeof(si));
  si.sin_family = AF_INET;
  si.sin_port = htons(PORT);
  si.sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(desc , (struct sockaddr*)&si, sizeof(si) ) == -1) {
    ROS_ERROR("[GUI_Node] socket bind die");
  }
} // End of init_socket

void execute_socket() {
  int recv_len;          // receive length
  int slen = sizeof(si); // si length
  char buffer[BUFFER_LENGTH];
  fflush(stdout);

  //try to receive some data, this is a blocking call
  if ((recv_len = recvfrom(desc, buffer, BUFFER_LENGTH, 0, (struct sockaddr *) &si,(socklen_t *) &slen)) == -1) {
    ROS_ERROR("[GUI_Node] socket recvfrom die");
  }

  printf("0x%x 0x%x 0x%x 0x%x\n", buffer[0], buffer[1], buffer[2], buffer[3]);

  //now reply the client with the same data
  if(buffer[0] == 0x64){
	ROS_INFO("Robot Forward Move !");
	cmd_vel.linear.x  = 0.2;
	cmd_vel.linear.y  = 0.0;
	cmd_vel.angular.z = 0.0;
  }
  else if(buffer[0] == 0x65){
	ROS_INFO("Robot Backward Move !");
	cmd_vel.linear.x  = -0.2;
	cmd_vel.linear.y  =  0.0;
	cmd_vel.angular.z =  0.0;
  }
  else if(buffer[0] == 0x66){
	ROS_INFO("Robot Turn Left");
	cmd_vel.linear.x  = 0.0;
	cmd_vel.linear.y  = 0.0;
	cmd_vel.angular.z = 0.6;
  }
  else if(buffer[0] == 0x67){
	ROS_INFO("Robot Turn Right");
	cmd_vel.linear.x  =  0.0;
	cmd_vel.linear.y  =  0.0;
	cmd_vel.angular.z = -0.6;
  }
  else {
	cmd_vel.linear.x  =  0.0;
	cmd_vel.linear.y  =  0.0;
	cmd_vel.angular.z =  0.0;
  }

  move_pub.publish(cmd_vel);

} // End of execute socket

int main(int argc, char** argv) {

  ros::init(argc, argv, "gui_node");
  ros::NodeHandle nh;
  move_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ROS_INFO("[GUI_Node] launch ...");

  init_socket();

  ROS_INFO("[GUI_Node] init socket ...");
  sleep(2);

  ros::Rate loop_rate(5);
  signal(SIGINT, shutdown);

  while (ros::ok()) {

	execute_socket(); // execute socket
    ros::spinOnce();
    loop_rate.sleep();

  } // End of ros::ok

  return 0;
}
