#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include <sstream>
#include <algorithm>

class SubFlipPub{
public:
  SubFlipPub(){
    flipped_pub = n.advertise<sensor_msgs::Image>("flipped_image", 100);    
    sub = n.subscribe("/camera/image_raw", 100, &SubFlipPub::flipCallback, this); 
  }

  void flipCallback(const sensor_msgs::Image::ConstPtr& img_msg)
  {
    flipped_msg = *img_msg; //copy ALL contents of msg
    for(int i=0; i < img_msg->height; i++){ //then flip just the data one row at a time
        
	std::copy ( img_msg->data.begin() + i * img_msg->step, 
                    img_msg->data.begin() + (i+1) * img_msg->step,
                    flipped_msg.data.end() - ((i+1) * img_msg->step) );
    }
    flipped_pub.publish(flipped_msg);
  }

private:
  sensor_msgs::Image flipped_msg;
  ros::NodeHandle n;
  ros::Publisher flipped_pub;
  ros::Subscriber sub;  
};


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * The third argument to init() is the name of the node.
   */
  ros::init(argc, argv, "flip");

  //Initializing node handle in class
  //ros::NodeHandle n;
  SubFlipPub sfp_object;

  ros::spin();


  return 0;
}
