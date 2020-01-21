#include <ros/ros.h>
#include <std_msgs/Float64.h>


# ifndef CPP_EXAMPLE_NODE
# define CPP_EXAMPLE_NODE


class Example
{
public:
  Example() {init();}
  ~Example() {}
  bool init();


private:
  ros::NodeHandle nh_;

  ros::Publisher reply_pub_;
  ros::Subscriber sub_;

  int response_step_;

  void python_pub_callback(const std_msgs::Float64);

};


# endif