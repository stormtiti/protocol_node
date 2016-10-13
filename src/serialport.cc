#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include "serialport.h"
#define BUFFER_LENGTH 1024  //Max length of buffer

#define FALSE  -1
#define TRUE   0
char buf[BUFFER_LENGTH];
char ackData;
int nread;
int iRet;
void *spReadHandle_thread(void* ptr);
SerialPort::SerialPort()
{
	int readthread;
	struct termios options_old, options;
	fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0) { /** error **/
		 printf("[%s-%d] open ttyS1 error!!!\n", __FILE__, __LINE__);
	}
	else
	{
	  printf("open ttyS1 success!!!\n");
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
	if ((readthread = pthread_create(&spRead_thread, NULL, spReadHandle_thread, (void*)(this))))
	{
		printf("readthread creation failed: %d\n", readthread);
	}
}

SerialPort::~SerialPort()
{
	close(fd);
}

void *spReadHandle_thread(void* ptr)
{

	SerialPort *me=(SerialPort*) ptr;
	struct timeval timeout;
	char buf[BUFFER_LENGTH];
	while(1)
	{
		FD_ZERO(&me->rd);
		FD_SET(me->fd,&me->rd);
	    timeout.tv_sec = 0;
	    timeout.tv_usec = 200000;//200ms
		switch (select(me->fd+1, &me->rd, NULL,NULL, &timeout))
		{
		  case -1:
		    printf("select err\n");
		    break;
		  case 0: //select timeout
		    me->data0 = 0xFF;
		    me->data1 = 0xFF;
		    break;
		  default:
		    if (FD_ISSET(me->fd,&me->rd)) {
		    	iRet = read(me->fd, buf, BUFFER_LENGTH);
				if(iRet > 0)
				{
					me->data0 = buf[0];
					me->data1 = buf[1];
				}
//		    	tcflush(me->fd,TCIFLUSH); // 清除正收到的数据，且不会读取出来。
		     }
		    break;
		}
	}
}
void SerialPort::writeData(unsigned char *data,int num)
{

	if(write(fd,data, num) < 0)
	{
		printf("write serialport failed \n");
	}
}

///*********************************************************************/
//int OpenDev()
//{
//	    int fd = open( "/dev/ttyS1", O_RDWR | O_NOCTTY );         //| O_NOCTTY | O_NDELAY
//	    if (-1 == fd)
//	    {
//	        perror("Can't Open Serial Port");
//	        return -1;
//	    }
//	    else
//	        return fd;
//}
//
//
///**
//*@brief  设置串口通信速率
//*@param  fd     类型 int  打开串口的文件句柄
//*@param  speed  类型 int  串口速度
//*@return  void
//*/
//int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
//	                    B38400, B19200, B9600, B4800, B2400, B1200, B300, };
//int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400,
//	                    19200,  9600, 4800, 2400, 1200,  300, };
//void set_speed(int fd, int speed)
//{
//	int   i;
//	int   status;
//	struct termios   Opt;
//	tcgetattr(fd, &Opt);
//	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
//		if  (speed == name_arr[i]) {
//			tcflush(fd, TCIOFLUSH);
//			cfsetispeed(&Opt, speed_arr[i]);
//			cfsetospeed(&Opt, speed_arr[i]);
//			status = tcsetattr(fd, TCSANOW, &Opt);
//			if  (status != 0) {
//				perror("tcsetattr fd1");
//				return;
//			}
//			tcflush(fd,TCIOFLUSH);
//		}
//	}
//}
///**
//*@brief   设置串口数据位，停止位和效验位
//*@param  fd     类型  int  打开的串口文件句柄
//*@param  databits 类型  int 数据位   取值 为 7 或者8
//*@param  stopbits 类型  int 停止位   取值为 1 或者2
//*@param  parity  类型  int  效验类型 取值为N,E,O,,S
//*/
//int set_Parity(int fd,int databits,int stopbits,int parity)
//{
//	struct termios options;
//	options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
//	options.c_oflag  &= ~OPOST;   /*Output*/
//	if  ( tcgetattr( fd,&options)  !=  0) {
//		perror("SetupSerial 1");
//		return(FALSE);
//	}
//	options.c_cflag &= ~CSIZE;
//	switch (databits) /*设置数据位数*/
//	{
//	case 7:
//		options.c_cflag |= CS7;
//		break;
//	case 8:
//		options.c_cflag |= CS8;
//		break;
//	default:
//		fprintf(stderr,"Unsupported data size\n"); return (FALSE);
//	}
//	switch (parity)
//	{
//		case 'n':
//		case 'N':
//			options.c_cflag &= ~PARENB;   /* Clear parity enable */
//			options.c_iflag &= ~INPCK;     /* Enable parity checking */
//			break;
//		case 'o':
//		case 'O':
//			options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
//			options.c_iflag |= INPCK;             /* Disnable parity checking */
//			break;
//		case 'e':
//		case 'E':
//			options.c_cflag |= PARENB;     /* Enable parity */
//			options.c_cflag &= ~PARODD;   /* 转换为偶效验*/
//			options.c_iflag |= INPCK;       /* Disnable parity checking */
//			break;
//		case 'S':
//		case 's':  /*as no parity*/
//			options.c_cflag &= ~PARENB;
//			options.c_cflag &= ~CSTOPB;break;
//		default:
//			fprintf(stderr,"Unsupported parity\n");
//			return (FALSE);
//	}
//	/* 设置停止位*/
//	switch (stopbits)
//	{
//		case 1:
//			options.c_cflag &= ~CSTOPB;
//			break;
//		case 2:
//			options.c_cflag |= CSTOPB;
//		   break;
//		default:
//			 fprintf(stderr,"Unsupported stop bits\n");
//			 return (FALSE);
//	}
//	/* Set input parity option */
//	if (parity != 'n')
//		options.c_iflag |= INPCK;
//	tcflush(fd,TCIFLUSH);
//	options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
//	options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
//	if (tcsetattr(fd,TCSANOW,&options) != 0)
//	{
//		perror("SetupSerial 3");
//		return (FALSE);
//	}
//	return (TRUE);
//}







