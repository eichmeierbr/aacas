#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

// #include <pcl/point_types.h>
// #include <pcl/PCLPointCloud2.h>
// // #include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>

// #include <velodyne_msgs/VelodyneScan.h>
#include <string>
// #include < vector> 

#include <typeinfo>
using namespace std;


// string tmp_lidar_msg; 

// vector<velodyne_msgs::VelodynePacket_<std::allocator<void> > tmp_lidar_msg;
// auto tmp_lidar_msg;
// int[1206] data;


// void lidar_msg_callback(const velodyne_msgs::VelodyneScan msg)
// {
//     auto tmp_lidar_msg = msg.packets.data;
// //  ROS_INFO("I heard: [%s]", tmp_msg.c_str());
// //  ROS_INFO("I heard: [%s]", tmp_lidar_msg.c_str());
// }


void cloud_callback(const sensor_msgs::PointCloud2 &msg){
    pcl::PointCloud<pcl::PointXYZ>  cloud_temp;
    pcl::fromROSMsg(msg, cloud_temp);
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "lidar_data_process");

    //   ros::NodeHandle n_pub;
    ros::NodeHandle n;

    //Add your code here
    // ros::Publisher chatter_pub = n.advertise<chatbot_node::reply_msg>("reply_msg", 1000);
    ros::Subscriber sub = n.subscribe("velodyne_points", 1000, lidar_msg_callback);
    ros::Rate loop_rate(20);

    // ros::spin();

  while(ros::ok()) {

    //   cout << tmp_lidar_msg << endl;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}    



// #include <iostream>
// // ROS headers
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// // point cloud definition
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// // namspace
// using namespace std;
// using namespace sensor_msgs;

    
// void callback(const sensor_msgs::PointCloud2ConstPtr& pCloud)
// {
// // new cloud formation 
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::fromROSMsg (*pCloud, *cloud);

// // cloud is the one you are interested about.
// }


// int main(int argc, char** argv)
// {
// ros::init(argc, argv, "lidar_data_process");

// ros::NodeHandle nh;
// ros::Rate rate(30); // frequency of operation

// // subscribe
// ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, callback);


// ros::spin();
// return 0;
// }