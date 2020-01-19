#include <ros/ros.h>
#include <string>
#include <std_msgs/Float64.h>
#include "cppExample.h"


//Add your code here


bool Example::init()
{
  // initialize publisher
  sub_ = nh_.subscribe("python_pub", 1000, &Example::python_pub_callback,this);
  
  reply_pub_ = nh_.advertise<std_msgs::Float64>("cpp_pub",1000);

  // Load Param
//   nh_.getParam("name",name_);
  return true;
}



void  Example::python_pub_callback(const std_msgs::Float64 msg)
{
    std_msgs::Float64 response;
    response.data = msg.data +1;
    reply_pub_.publish(response);

}




int main(int argc, char **argv) {

  ros::init(argc, argv, "cpp_node");

  //Add your code here
  Example chat_node;

  ros::Rate loop_rate(20);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}