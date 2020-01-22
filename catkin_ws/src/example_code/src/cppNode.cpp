#include <ros/ros.h>
#include <string>
#include <std_msgs/Float64.h>
#include "cppExample.h"


//Add your code here


bool Example::init()
{
  // Load Params
//   nh_.getParam(ROSPARAM_NAME,VARIABLE NAME);

    std::string py_pub;
    std::string cpp_pub;
    nh_.getParam("py_pub_name",py_pub);
    nh_.getParam("cpp_pub_name",cpp_pub);
    nh_.getParam("cpp_response_step",response_step_);

  // Initialize Subscriber
  // class_var = nh_.subscribe(TOPIC_NAME, QUEUE SIZE, CALLBACK_FUNC, this)
  sub_ = nh_.subscribe(py_pub, 1000, &Example::python_pub_callback,this);

  // Initialize Publisher
  // class_var = nh_.advertise<DATA_TYPE>(TOPIC NAME, QUEUE SIZE)
  reply_pub_ = nh_.advertise<std_msgs::Float64>(cpp_pub,1000);

  return true;
}



void  Example::python_pub_callback(const std_msgs::Float64 msg)
{
    std_msgs::Float64 response;
    response.data = msg.data + response_step_;
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