#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

/*#include "sensor_msgs/Imu.h"
void callBack(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->orientation.x);
}
*/

void callBack(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->pose.pose.position.x);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reader");
  ros::NodeHandle n;
  ros::Subscriber subImu = n.subscribe("/ardrone/odometry", 1000, callBack);
  ros::spin();
}
