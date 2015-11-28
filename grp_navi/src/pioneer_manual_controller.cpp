#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c
#define KEYCODE_Q 0x71
#define KEYCODE_Z 0x7a
#define KEYCODE_W 0x77
#define KEYCODE_X 0x78
#define KEYCODE_E 0x65
#define KEYCODE_C 0x63
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_M 0x6d
#define KEYCODE_R 0x72
#define KEYCODE_V 0x76
#define KEYCODE_S 0x73
#define KEYCODE_T 0x74
#define KEYCODE_B 0x62
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_X 0x78

#define KEYCODE_COMMA 0x2c
#define KEYCODE_PERIOD 0x2e

#define COMMAND_TIMEOUT_SEC 0.2

// at full joystick depression you'll go this fast
double max_speed = 0.050; // m/second
double max_turn = 5.0*M_PI/180.0; // rad/second
// should we continuously send commands?
bool always_command = false;


class TBK_Node
{
private:
	geometry_msgs::Twist cmdvel;
	ros::NodeHandle n_;
	ros::Publisher pub_;

public:
	TBK_Node()
	{
		pub_ = n_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);
	}
	~TBK_Node() { }
	void keyboardLoop();
	void stopRobot()
	{
		cmdvel.linear.x = cmdvel.angular.z = 0.0;
		pub_.publish(cmdvel);
	}
};

TBK_Node* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	TBK_Node tbk;

	boost::thread t = boost::thread(boost::bind(&TBK_Node::keyboardLoop, &tbk));

	ros::spin();

	t.interrupt();
	t.join();
	tbk.stopRobot();
	tcsetattr(kfd, TCSANOW, &cooked);

	return(0);
}

void
	TBK_Node::keyboardLoop()
{
	char keyboard_input;
	double max_tv = max_speed;
	double max_rv = max_turn;
	bool dirty=false;

	int speed=0;
	int turn=0;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("q/z : increase/decrease max angular and linear speeds by 10%");
	puts("w/x : increase/decrease max linear speed by 10%");
	puts("e/c : increase/decrease max angular speed by 10%");
	puts("---------------------------");
	puts("Moving around:");
	puts("   u    i    o");
	puts("   j    k    l");
	puts("   m    ,    .");
	puts("anything else : stop");
	puts("---------------------------");

	struct pollfd ufd;
	ufd.fd = kfd;
	ufd.events = POLLIN;
	for(;;)
	{
		double fast_v = 3;
		double fast_w = 3;
		boost::this_thread::interruption_point();

		// get the next event from the keyboard
		int num;
		if((num = poll(&ufd, 1, 250)) < 0)
		{
			perror("poll():");
			return;
		}
		else if(num > 0)
		{
			if(read(kfd, &keyboard_input, 1) < 0)
			{
				perror("read():");
				return;
			}
		}
		else
			continue;


		// TODO
		// ASCII CODE of keyboard input is contained in variable c
		// refer above defined acronym for ASCII codes, write a code that publishing
		// adequate control of pioneer which coincides with keyboard input.
		//
		// 1. make control when specific key is pressed.
		// 2. publish control
		// if you follow instruction carefully, then this is so easy.
		switch(keyboard_input)
		{
		case KEYCODE_I:
			cmdvel.linear.x = max_tv;
			cmdvel.angular.z = 0;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_COMMA:
			cmdvel.linear.x = -max_tv;
			cmdvel.angular.z = 0;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_J:
			cmdvel.linear.x = 0;
			cmdvel.angular.z = -max_rv;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_O:
			cmdvel.linear.x = max_tv;
			cmdvel.angular.z = max_rv;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;	
		case KEYCODE_L:
			cmdvel.linear.x = 0;
			cmdvel.angular.z = max_rv;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_U:
			cmdvel.linear.x = max_tv;
			cmdvel.angular.z = -max_rv;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_M:
			cmdvel.linear.x = -max_tv;
			cmdvel.angular.z = max_rv;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_PERIOD:
			cmdvel.linear.x = -max_tv;
			cmdvel.angular.z = -max_rv;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_K:
			cmdvel.linear.x = 0;
			cmdvel.angular.z = 0;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_Q:
			max_rv = max_rv*1.1;
			max_tv = max_tv*1.1;
			cmdvel.linear.x *= 1.1;
			cmdvel.angular.z *= 1.1;		
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_Z:
			max_rv = max_rv*0.9;
			max_tv = max_tv*0.9;

			cmdvel.linear.x *= 0.9;
			cmdvel.angular.z *= 0.9;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_W:
			max_tv = max_tv*1.1;

			cmdvel.linear.x *= 1.1;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_X:
			max_tv = max_tv*0.9;

			cmdvel.linear.x *= 0.9;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_E:
			max_rv = max_rv*1.1;
			cmdvel.angular.z *= 1.1;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		case KEYCODE_C:
			max_rv = max_rv*0.9;
			cmdvel.angular.z *= 0.9;
			pub_.publish(cmdvel);
			ros::spinOnce();
			break;
		default:
			cmdvel.angular.z = 0;
			cmdvel.linear.x = 0;
			pub_.publish(cmdvel);
			ros::spinOnce();
		}



	}
}
