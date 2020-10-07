#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <string>
#include <vector>

geometry_msgs::Quaternion geom_quat;
nav_msgs::Path path;
float transform_z_offset;

void attitudeCallback(const geometry_msgs::QuaternionStamped& msg){
  geom_quat = msg.quaternion;

}


void poseCallback(const geometry_msgs::PointStamped& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  // Create Translation
  auto x = msg.point.x;
  auto y = msg.point.y;
  auto z = msg.point.z;
  transform.setOrigin( tf::Vector3(x,y,z) );

  // Create Quaternion
  tf::Quaternion q(geom_quat.x, geom_quat.y, geom_quat.z, geom_quat.w);
  q.normalize();
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "vehicle_center_link"));

  // Update and Send Path
  path.header = msg.header;
  path.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  pose.header = msg.header;
  pose.header.frame_id = "world";
  pose.pose.position = msg.point;
  pose.pose.orientation = geom_quat;
  path.poses.push_back(pose);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;
  ros::Publisher path_pub = node.advertise<nav_msgs::Path>("path", 10);


  std::string position_topic, attitude_topic;

  node.getParam("position_pub_name", position_topic);
  node.getParam("attitude_pub_name", attitude_topic);
  node.getParam("transform_z_offset", transform_z_offset);

  ros::Subscriber pos_sub = node.subscribe(position_topic, 10, &poseCallback);
  ros::Subscriber att_sub = node.subscribe(attitude_topic, 10, &attitudeCallback);


  ros::Rate loop_rate(10);
  while(path.poses.size() < 1){
    ros::spinOnce();
    loop_rate.sleep();
  }
  while(ros::ok())
  {
    path_pub.publish(path);
    ros::spinOnce();
    loop_rate.sleep();
    
  }

  ros::spin();
  return 0;
};