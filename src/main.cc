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
#include <stdlib.h>     /*标准函数库定义*/
#include <sys/types.h>
#include <sys/stat.h>

#define BUFFER_LENGTH 1024  //Max length of buffer

#define FALSE  -1
#define TRUE   0


int fd; /* File descriptor for the port */
geometry_msgs::Twist cmd_vel;
ros::Publisher move_pub;

void shutdown(int sig) {
  ROS_INFO("[GUI_Node] disconnect gui node ...");

  close(fd);
  ros::shutdown();
}
/*********************************************************************/
int OpenDev()
{
	    int fd = open( "/dev/ttyS1", O_RDWR | O_NOCTTY );         //| O_NOCTTY | O_NDELAY
	    if (-1 == fd)
	    {
	        perror("Can't Open Serial Port");
	        return -1;
	    }
	    else
	        return fd;
}

/**
*@brief  设置串口通信速率
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*@return  void
*/
int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
	                    B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400,
	                    19200,  9600, 4800, 2400, 1200,  300, };
void set_speed(int fd, int speed)
{
	int   i;
	int   status;
	struct termios   Opt;
	tcgetattr(fd, &Opt);
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
		if  (speed == name_arr[i]) {
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed_arr[i]);
			cfsetospeed(&Opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &Opt);
			if  (status != 0) {
				perror("tcsetattr fd1");
				return;
			}
			tcflush(fd,TCIOFLUSH);
		}
	}
}

/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄
*@param  databits 类型  int 数据位   取值 为 7 或者8
*@param  stopbits 类型  int 停止位   取值为 1 或者2
*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*/
int set_Parity(int fd,int databits,int stopbits,int parity)
{
	struct termios options;
	options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
	options.c_oflag  &= ~OPOST;   /*Output*/
	if  ( tcgetattr( fd,&options)  !=  0) {
		perror("SetupSerial 1");
		return(FALSE);
	}
	options.c_cflag &= ~CSIZE;
	switch (databits) /*设置数据位数*/
	{
	case 7:
		options.c_cflag |= CS7;
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		fprintf(stderr,"Unsupported data size\n"); return (FALSE);
	}
	switch (parity)
	{
		case 'n':
		case 'N':
			options.c_cflag &= ~PARENB;   /* Clear parity enable */
			options.c_iflag &= ~INPCK;     /* Enable parity checking */
			break;
		case 'o':
		case 'O':
			options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
			options.c_iflag |= INPCK;             /* Disnable parity checking */
			break;
		case 'e':
		case 'E':
			options.c_cflag |= PARENB;     /* Enable parity */
			options.c_cflag &= ~PARODD;   /* 转换为偶效验*/
			options.c_iflag |= INPCK;       /* Disnable parity checking */
			break;
		case 'S':
		case 's':  /*as no parity*/
			options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;break;
		default:
			fprintf(stderr,"Unsupported parity\n");
			return (FALSE);
	}
	/* 设置停止位*/
	switch (stopbits)
	{
		case 1:
			options.c_cflag &= ~CSTOPB;
			break;
		case 2:
			options.c_cflag |= CSTOPB;
		   break;
		default:
			 fprintf(stderr,"Unsupported stop bits\n");
			 return (FALSE);
	}
	/* Set input parity option */
	if (parity != 'n')
		options.c_iflag |= INPCK;
	tcflush(fd,TCIFLUSH);
	options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
	options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
	if (tcsetattr(fd,TCSANOW,&options) != 0)
	{
		perror("SetupSerial 3");
		return (FALSE);
	}
	return (TRUE);
}




int main(int argc, char** argv) {

  ros::init(argc, argv, "gui_node");
  signal(SIGINT, shutdown);

  ros::NodeHandle nh;
  move_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ROS_INFO("[GUI_Node] launch ...");

  char buf[BUFFER_LENGTH];
  int nread;
  int iRet;
  struct termios options_old, options;
  fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0) { /** error **/
         printf("[%s-%d] open error!!!\n", __FILE__, __LINE__);
  }
  else
  {
	  printf("open success!!!\n");
  }
  fcntl(fd, F_SETFL, 0);
  /** * Get the current options for the port... **/
  tcgetattr(fd, &options);
  options_old = options;
  /*** Set the baud rates to 19200... **/
  cfsetispeed(&options, B19200);
  cfsetospeed(&options, B19200);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input*/
  //options.c_oflag |= OPOST; /** 加工过的输出 **/
  options.c_oflag &= ~OPOST; /** 选择原始输出 **/
  /*** Enable the receiver and set local mode... **/
  // options.c_cflag |= (CLOCAL | CREAD);
  /*** Set the new options for the port... **/
  tcsetattr(fd, TCSANOW, &options);
  sleep(2);
  ros::Rate loop_rate(5);
  while (ros::ok()) {

    char buf[BUFFER_LENGTH];
//    ROS_INFO("write buffer----------\n");
    iRet = read(fd, buf, 1024);
    if(iRet > 0)
    {
    	printf("rec length %d \n",iRet);
    	printf("0x%x, 0x%x, 0x%x, 0x%x\n", buf[0], buf[1], buf[2], buf[3]);
		//now reply the client with the same data
		if(buf[0] == 0x61){
		  ROS_INFO("Robot Forward Move !");
		  cmd_vel.linear.x  = 0.2;
		  cmd_vel.linear.y  = 0.0;
		  cmd_vel.angular.z = 0.0;
		}
		else if(buf[0] == 0x62){
		  ROS_INFO("Robot Backward Move !");
		  cmd_vel.linear.x  = -0.2;
		  cmd_vel.linear.y  =  0.0;
		  cmd_vel.angular.z =  0.0;
		}
		else if(buf[0] == 0x63){
		  ROS_INFO("Robot Turn Left");
		  cmd_vel.linear.x  = 0.0;
		  cmd_vel.linear.y  = 0.0;
		  cmd_vel.angular.z = 0.6;
		}
		else if(buf[0] == 0x64){
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
    }
    else
    {
    	printf("receive err \n");
    	cmd_vel.linear.x  =  0.0;
    	cmd_vel.linear.y  =  0.0;
    	cmd_vel.angular.z =  0.0;
    }
//    buf[0] = 71;
//    buf[1] = 72;
//    buf[2] = 73;
//    buf[3] = 74;
//    buf[4] = 75;
//    buf[5] = 76;
//    buf[6] = 77;
//    buf[7] = 78;
//    nread = write(fd, buf ,8);
//    if(nread == -1)
//    {
//    	printf("Wirte sbuf error.\n");
//    }
//    else
//    {
//    	printf("Wirte sbuf success.\n");
//    }

    move_pub.publish(cmd_vel);
    fflush(stdout);
    ros::spinOnce();
    loop_rate.sleep();

  } // End of ros::ok
  return 0;
}
