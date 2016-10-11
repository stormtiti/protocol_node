#ifndef SERIALPORT_H
#define SERIALPORT_H
class SerialPort
{
public:
	SerialPort();
    ~SerialPort();

	/// Set Receive
	/// AcktoAndriod
	int fd; /* File descriptor for the port */
	void writeData(unsigned char *data,int num);
	char data0,data1;

	pthread_t spRead_thread;
};
#endif // CANSEND_H
