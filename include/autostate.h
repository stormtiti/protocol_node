#ifndef AUTOSTATE_H
#define AUTOSTATE_H
#define MANUAL 				0x60
#define MANUAL_FORWARD		0x61
#define MANUAL_BACKWARD		0x62
#define MANUAL_LEFT			0x63
#define MANUAL_RIGHT		0x64
#define MANUAL_STOP			0x65

#define AUTO_NAVI 			0x70
#define GOTO_STATE 			0x71
#define GOTO_ACK_PROCESS	0x72
#define GOTO_ACK_ARRIVE		0x73
#define PAUSE_STATE 		0x80
#define TERMINATE_STATE 	0x90

typedef struct{
	double x;
	double y;
	double a;
}Pos;
class AutoState
{
public:
	AutoState();
    ~AutoState();

    unsigned char State;
    unsigned char StateBack;
    unsigned char manualState;
    unsigned char naviState;
    unsigned char naviStateBack;
    unsigned char goto_aim;
    unsigned char goto_aimBak;

    unsigned char state_cmd;
    unsigned char state_dest;

    unsigned char navAck;
	pthread_t AutoState_thread;
	geometry_msgs::Twist cmd_vel;
	geometry_msgs::PoseWithCovarianceStamped robot_pose;
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Publisher Goal_pub;
	ros::Publisher Pause_pub;
	ros::Publisher Init_pos_pub;

	void setCurrobotPos(geometry_msgs::PoseWithCovarianceStamped acml_msgs);
	void SendStop();
	void SendInitPos(float x, float y,float a);
	/// Set Receive
	void setReceive(unsigned char cmd, unsigned char dest);
	/// AcktoAndriod
	int AcktoAndriod(unsigned char &ack,unsigned char &dest);

	void SendNextPos(Pos ps);


};
#endif // CANSEND_H
