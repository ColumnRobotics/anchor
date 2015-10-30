//Ardrone Mover Node 
//Job Beford


#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <std_msgs/Char.h>
//#include "anchor_drone/KeyCode.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Empty.h>
#include <iostream>

#define KEYCODE_SPACE 0x20
#define KEYCODE_ESC 0x1B
#define KEYCODE_s 0x73
#define LATERAL_SPEED 0.1
#define ROTATION_SPEED 0.2

using namespace std;

class AnchorDrone
{
public:
  AnchorDrone();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  //ros::Publisher key_pub_;
  ros::Publisher land_pub_;
  ros::Publisher reset_pub_;
  ros::Publisher takeoff_pub_;
  ros::Publisher flattrim_pub_;
  ros::Publisher cmd_pub_;
  geometry_msgs::Twist cmd;//
  //geometry_msgs::TwistStamped cmd;
};

//Sets up Publishers
AnchorDrone::AnchorDrone()
{
  //key_pub_ = nh_.advertise<anchor_Drone::KeyCode.("keycode", 10);
  //cmd_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/teleop/cmd_vel", 10);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  land_pub_ = nh_.advertise<std_msgs::Empty>("/ardrone/land", 10);
  reset_pub_ = nh_.advertise<std_msgs::Empty>("/ardrone/reset", 10);
  takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 10);
  flattrim_pub_ = nh_.advertise<std_msgs::Empty>("/ardrone/flattrim", 10);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "anchor_drone");
  AnchorDrone anchor_drone;

  signal(SIGINT, quit);

  anchor_drone.keyLoop();

  return(0);
}


void AnchorDrone::keyLoop()
{
  std_msgs::Empty empty;
  char c;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);



  puts("Reading from keyboard");
  puts("--------------------------");
  puts("Reading from the keyboard and Publishing to Twist!");
  puts("--------------------------");
  puts("up/down:       move forward/backward");
  puts("left/right:    move left/right");
  puts("w/s:           increase/decrease altitude");
  puts("a/d:           turn left/right");
  puts("t/l:           takeoff/land");
  puts("r:             reset (toggle emergency state)");
  puts("f:             flat trim");
  puts("i:             enable PTAM auto init");
  puts("anything else: stop");

  puts("please don't have caps lock on.");
  puts("CTRL+c to quit");
  //puts("Use arrow keys to move the drone.");


  for(;;)
    {

      if(read(kfd, &c, 1) < 0)
	{
	  perror("read():");
	  exit(-1);
	}

      //cmd.header.stamp = ros::Time::now();

      switch(c)
	{ 
	case 'l':
	  cout<<"landing"<<endl;
	  land_pub_.publish(empty);
	  break;
	case 'r':
	  cout<<"Reset"<<endl;
	  reset_pub_.publish(empty);
	  break;
	case 't':
	  cout<<"TakeOff"<<endl;
	  takeoff_pub_.publish(empty);
       	  break;
	case 'f':
	  cout<<"Flat trim"<<endl;
	  flattrim_pub_.publish(empty);
      	  break;
		  
	case 0x41:
	  cout<<"Forward"<<endl;
	  /*
	  cmd.header.stamp = ros::Time::now();
	  cmd.twist.linear.x  = LATERAL_SPEED;
	  cmd.twist.linear.y  = 0;
	  cmd.twist.linear.z  = 0;
	  cmd.twist.angular.x = 0;
	  cmd.twist.angular.y = 0;
	  cmd.twist.angular.z = 0;
	  */
	  //cmd.header.stamp = ros::Time::now();
	  cmd.linear.x  = LATERAL_SPEED;
	  cmd.linear.y  = 0;
	  cmd.linear.z  = 0;
	  cmd.angular.x = 0;
	  cmd.angular.y = 0;
	  cmd.angular.z = 0;

	  cmd_pub_.publish(cmd);
	  break;

	case 0x42:
	  cout<<"Backward"<<endl;
	  /*
	  cmd.header.stamp = ros::Time::now();
	  cmd.twist.linear.x  = -LATERAL_SPEED;
	  cmd.twist.linear.y  = 0;
	  cmd.twist.linear.z  = 0;
	  cmd.twist.angular.x = 0;
	  cmd.twist.angular.y = 0;
	  cmd.twist.angular.z = 0;*/
	  //cmd.header.stamp = ros::Time::now();
	  
	  cmd.linear.x  = -LATERAL_SPEED;
	  cmd.linear.y  = 0;
	  cmd.linear.z  = 0;
	  cmd.angular.x = 0;
	  cmd.angular.y = 0;
	  cmd.angular.z = 0;

	  cmd_pub_.publish(cmd);
	  
	  break;

	case 0x43:
	  cout<<"Right"<<endl;
	  /*cmd.header.stamp = ros::Time::now();
	  cmd.twist.linear.x  = 0;
	  cmd.twist.linear.y  = -LATERAL_SPEED;
	  cmd.twist.linear.z  = 0;
	  cmd.twist.angular.x = 0;
	  cmd.twist.angular.y = 0;
	  cmd.twist.angular.z = 0;
	  cmd_pub_.publish(cmd);*/
	  cmd.linear.x  = 0;
	  cmd.linear.y  = -LATERAL_SPEED;
	  cmd.linear.z  = 0;
	  cmd.angular.x = 0;
	  cmd.angular.y = 0;
	  cmd.angular.z = 0;
	  cmd_pub_.publish(cmd);
	  break;
	  
	case 0x44:
	  cout<<"Left"<<endl;
	  /*cmd.header.stamp = ros::Time::now();
	  cmd.twist.linear.x  = 0;
	  cmd.twist.linear.y  = LATERAL_SPEED;
	  cmd.twist.linear.z  = 0;
	  cmd.twist.angular.x = 0;
	  cmd.twist.angular.y = 0;
	  cmd.twist.angular.z = 0;
	  cmd_pub_.publish(cmd);*/
	  cmd.linear.x  = 0;
	  cmd.linear.y  = LATERAL_SPEED;
	  cmd.linear.z  = 0;
	  cmd.angular.x = 0;
	  cmd.angular.y = 0;
	  cmd.angular.z = 0;
	  cmd_pub_.publish(cmd);
	  break;

	case 'w':
	  cout<<"Up"<<endl;
	  /*cmd.header.stamp = ros::Time::now();
	  cmd.twist.linear.x  = 0;
	  cmd.twist.linear.y  = 0;
	  //cmd.twist.linear.z  = LATERAL_SPEED;
	  cmd.twist.linear.z  = LATERAL_SPEED;
	  cmd.twist.angular.x = 0;
	  cmd.twist.angular.y = 0;
	  cmd.twist.angular.z = 0;
	  cmd_pub_.publish(cmd);*/
	  cmd.linear.x  = 0;
	  cmd.linear.y  = 0;
	  cmd.linear.z  = LATERAL_SPEED;
	  cmd.angular.x = 0;
	  cmd.angular.y = 0;
	  cmd.angular.z = 0;
	  cmd_pub_.publish(cmd);
	  break;
	  
	case 's':
	  cout<<"Down"<<endl;
	  /*cmd.header.stamp = ros::Time::now();
	  cmd.twist.linear.x  = 0;
	  cmd.twist.linear.y  = 0;
	  cmd.twist.linear.z  = -LATERAL_SPEED;
	  cmd.twist.angular.x = 0;
	  cmd.twist.angular.y = 0;
	  cmd.twist.angular.z = 0;
	  cmd_pub_.publish(cmd);*/
	  cmd.linear.x  = 0;
	  cmd.linear.y  = 0;
	  cmd.linear.z  = -LATERAL_SPEED;
	  cmd.angular.x = 0;
	  cmd.angular.y = 0;
	  cmd.angular.z = 0;
	  cmd_pub_.publish(cmd);
	  break;
	  
	case 'a':
	  cout<<"Left rotation"<<endl;
	  /*cmd.header.stamp = ros::Time::now();
	  cmd.twist.linear.x  = 0;
	  cmd.twist.linear.y  = 0;
	  cmd.twist.linear.z  = 0;
	  cmd.twist.angular.x = 0;
	  cmd.twist.angular.y = 0;
	  cmd.twist.angular.z = ROTATION_SPEED;
	  //cmd.twist.angular.z = 0.4;*/
	  cmd.linear.x  = 0;
	  cmd.linear.y  = 0;
	  cmd.linear.z  = 0;
	  cmd.angular.x = 0;
	  cmd.angular.y = 0;
	  cmd.angular.z = ROTATION_SPEED;
	  cmd_pub_.publish(cmd);
	  break;
	  
	case 'd':
	  cout<<"Right rotation"<<endl;
	  /*cmd.header.stamp = ros::Time::now();
	  cmd.twist.linear.x  = 0;
	  cmd.twist.linear.y  = 0;
	  cmd.twist.linear.z  = 0;
	  cmd.twist.angular.x = 0;
	  cmd.twist.angular.y = 0;
	  cmd.twist.angular.z = -ROTATION_SPEED;
	  //cmd.twist.angular.z = -0.4;*/
	  cmd.linear.x  = 0;
	  cmd.linear.y  = 0;
	  cmd.linear.z  = 0;
	  cmd.angular.x = 0;
	  cmd.angular.y = 0;
	  cmd.angular.z = -ROTATION_SPEED;
	  cmd_pub_.publish(cmd);
	  break;

	case 'g':
	  cout<<"Spiral"<<endl;
	  cmd.linear.x  = 0;
	  cmd.linear.y  = -.4;
	  cmd.linear.z  = 0;
	  cmd.angular.x = 0;
	  cmd.angular.y = 0;
	  cmd.angular.z = -1.0;
	  cmd_pub_.publish(cmd);
	  break;

	case 'h':
	  cout<<"halt"<<endl;
	  cmd.linear.x  = 0;
	  cmd.linear.y  = 0;
	  cmd.linear.z  = 0;
	  cmd.angular.x = 0;
	  cmd.angular.y = 0;
	  cmd.angular.z = 0;
	  cmd_pub_.publish(cmd);
	  break;
	  
	case 0x1B:
	  break;
	  
	case 0x5B:
	  break;
	  
	default:
	  /*cmd.header.stamp = ros::Time::now();
	  cmd.twist.linear.x  = 0;
	  cmd.twist.linear.y  = 0;
	  cmd.twist.linear.z  = 0;
	  cmd.twist.angular.x = 0;
	  cmd.twist.angular.y = 0;
	  cmd.twist.angular.z = 0;*/

	  //cmd.header.stamp = ros::Time::now();
	  cmd.linear.x  = 0;
	  cmd.linear.y  = 0;
	  cmd.linear.z  = 0;
	  cmd.angular.x = 0;
	  cmd.angular.y = 0;
	  cmd.angular.z = 0;

	  cmd_pub_.publish(cmd);
	  break;
	}




      //anchor_drone::KeyCode code;
      //code.data = c;
      //key_pub_.publish(code);
      usleep(10);
    }
  return;
}
