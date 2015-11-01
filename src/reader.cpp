#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

void callBack(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Quaternion: x:%f y:%f z:%f w:%f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reader");
  ros::NodeHandle n;
  ros::Subscriber subImu = n.subscribe("/ardrone/imu", 1000, callBack);
  ros::spin();
}
