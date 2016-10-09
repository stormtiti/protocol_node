/*
 * main.cc
 *
 *  Created on: Oct 8, 2016
 *      Author: root
 */

// ros library usage
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
// c++ std lib
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#define BUFFER_LENGTH 1024  //Max length of buffer
int fd; /* File descriptor for the port */
geometry_msgs::Twist cmd_vel;
ros::Publisher move_pub;

void shutdown(int sig) {
  ROS_INFO("[GUI_Node] disconnect gui node ...");
  close(fd);
  ros::shutdown();
}

//void execute_socket() {
//  int recv_len;          // receive length
//  int slen = sizeof(si); // si length
//  char buffer[BUFFER_LENGTH];
//  fflush(stdout);
//
//  //try to receive some data, this is a blocking call
//  if ((recv_len = recvfrom(desc, buffer, BUFFER_LENGTH, 0, (struct sockaddr *) &si,(socklen_t *) &slen)) == -1) {
//    ROS_ERROR("[GUI_Node] socket recvfrom die");
//  }
//
//  printf("0x%x 0x%x 0x%x 0x%x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
//
//  //now reply the client with the same data
//  if(buffer[0] == 0x64){
//	ROS_INFO("Robot Forward Move !");
//	cmd_vel.linear.x  = 0.2;
//	cmd_vel.linear.y  = 0.0;
//	cmd_vel.angular.z = 0.0;
//  }
//  else if(buffer[0] == 0x65){
//	ROS_INFO("Robot Backward Move !");
//	cmd_vel.linear.x  = -0.2;
//	cmd_vel.linear.y  =  0.0;
//	cmd_vel.angular.z =  0.0;
//  }
//  else if(buffer[0] == 0x66){
//	ROS_INFO("Robot Turn Left");
//	cmd_vel.linear.x  = 0.0;
//	cmd_vel.linear.y  = 0.0;
//	cmd_vel.angular.z = 0.6;
//  }
//  else if(buffer[0] == 0x67){
//	ROS_INFO("Robot Turn Right");
//	cmd_vel.linear.x  =  0.0;
//	cmd_vel.linear.y  =  0.0;
//	cmd_vel.angular.z = -0.6;
//  }
//  else {
//	cmd_vel.linear.x  =  0.0;
//	cmd_vel.linear.y  =  0.0;
//	cmd_vel.angular.z =  0.0;
//  }
//
//  move_pub.publish(cmd_vel);
//
//} // End of execute socket

int main(int argc, char** argv) {

  ros::init(argc, argv, "gui_node");
  signal(SIGINT, shutdown);

  ros::NodeHandle nh;
  move_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ROS_INFO("[GUI_Node] launch ...");

  char buf[BUFFER_LENGTH];
  fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0) { /** error **/
    ROS_ERROR("[%s-%d] open error!!!\n", __FILE__, __LINE__);
	return 0;
  }

  fcntl(fd, F_SETFL, 0);
  sleep(2);
  ros::Rate loop_rate(5);
  while (ros::ok()) {

    char buf[BUFFER_LENGTH];
    int iRet = read(fd, buf, 1024);
    printf("0x%x, 0x%x, 0x%x, 0x%x\n", buf[0], buf[1], buf[2], buf[3]);

    //now reply the client with the same data
    if(buf[0] == 0x64){
      ROS_INFO("Robot Forward Move !");
      cmd_vel.linear.x  = 0.2;
      cmd_vel.linear.y  = 0.0;
      cmd_vel.angular.z = 0.0;
    }
    else if(buf[0] == 0x65){
      ROS_INFO("Robot Backward Move !");
      cmd_vel.linear.x  = -0.2;
      cmd_vel.linear.y  =  0.0;
      cmd_vel.angular.z =  0.0;
    }
    else if(buf[0] == 0x66){
      ROS_INFO("Robot Turn Left");
      cmd_vel.linear.x  = 0.0;
      cmd_vel.linear.y  = 0.0;
      cmd_vel.angular.z = 0.6;
    }
    else if(buf[0] == 0x67){
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
    fflush(stdout);
    ros::spinOnce();
    loop_rate.sleep();

  } // End of ros::ok

  return 0;
}
