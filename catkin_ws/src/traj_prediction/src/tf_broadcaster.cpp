#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <string>
#include <vector>
#include <lidar_process/tracked_obj.h>
#include <lidar_process/tracked_obj_arr.h>
#include <traj_prediction/tracked_obj_arr.h>
#include "geometry_msgs/PointStamped.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


uint32_t shape = visualization_msgs::Marker::CUBE;
uint32_t drone_shape = visualization_msgs::Marker::CYLINDER;
geometry_msgs::Quaternion geom_quat;
nav_msgs::Path detected_path;
nav_msgs::Path predicted_path;
nav_msgs::Path transformed_future_path;

nav_msgs::Path drone_traj;
visualization_msgs::Marker drone_marker;
visualization_msgs::MarkerArray marker_array;
float transform_z_offset;

std::string fixed_frame = "drone_frame";

void attitude_cb(const geometry_msgs::QuaternionStamped& msg){
  geom_quat = msg.quaternion;

}

void drone_pos_cb(const::geometry_msgs::PointStamped msg ){
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

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", fixed_frame));

    //Drone Trajectory 
    drone_traj.header = msg.header;
    drone_traj.header.frame_id = fixed_frame;
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.header.frame_id = fixed_frame;
    pose.pose.position.x= msg.point.x;
    pose.pose.position.y = msg.point.y;
    pose.pose.position.z = msg.point.z;
    pose.pose.orientation = geom_quat;
    drone_traj.poses.push_back(pose);

    //Drone Marker
    drone_marker.header.frame_id = fixed_frame;
    drone_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    drone_marker.ns = "basic_shapes";
    drone_marker.id = 9999;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    drone_marker.type = drone_shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    drone_marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    drone_marker.pose.position.x = msg.point.x;
    drone_marker.pose.position.y = msg.point.y;
    drone_marker.pose.position.z = msg.point.z;

    drone_marker.pose.orientation = geom_quat ;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    drone_marker.scale.x = 1.0;
    drone_marker.scale.y = 1.0;
    drone_marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    drone_marker.color.r = 0.0f;
    drone_marker.color.g = 1.0f;
    drone_marker.color.b = 0.0f;
    drone_marker.color.a = 1.0;

    drone_marker.lifetime = ros::Duration();
    marker_array.markers.push_back(drone_marker);

}



void future_path_cb(nav_msgs::Path msg){
    transformed_future_path.poses.clear();
    for (int i=0 ; i<msg.poses.size();i++){
        auto x = msg.poses[i].pose.position.x;
        auto y = msg.poses[i].pose.position.y;
        auto z = msg.poses[i].pose.position.z;
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x,y,z));

        // Create Quaternion
        tf::Quaternion q(geom_quat.x, geom_quat.y, geom_quat.z, geom_quat.w);
        q.normalize();
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", fixed_frame));

        // Update and Send Path
        transformed_future_path.header = msg.header;
        transformed_future_path.header.frame_id = fixed_frame;
        geometry_msgs::PoseStamped pose;
        pose.header = msg.header;
        pose.header.frame_id = fixed_frame;
        pose.pose.position.x= msg.poses[i].pose.position.x;
        pose.pose.position.y = msg.poses[i].pose.position.y;
        pose.pose.position.z = msg.poses[i].pose.position.z;
        // pose.pose.orientation = geom_quat;
        transformed_future_path.poses.push_back(pose);
    }
}




void tracked_obj_cb(const lidar_process::tracked_obj_arr msg){
    for (auto& it : msg.tracked_obj_arr) {
        visualization_msgs::Marker marker;
        // visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = fixed_frame;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = it.object_id;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = it.point.x;
        marker.pose.position.y = it.point.y;
        marker.pose.position.z = it.point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(0.1);
        // marker_array.markers.push_back(marker);
        // for (int i=0; i <  marker_array.markers.size(); i++){
        //     std::cout<<"Remaining Marker ID" << marker_array.markers[i].id << std::endl;
        // }


        // std::cout << marker_array.markers.size() << std::endl;
        if (it.object_id == 0){
            // std::cout << "receiving new point" << std::endl;
            static tf::TransformBroadcaster br;
            tf::Transform transform;

            // Create Translation
            auto x = it.point.x;
            auto y = it.point.y;
            auto z = it.point.z;
            transform.setOrigin( tf::Vector3(x,y,z) );

            // Create Quaternion
            tf::Quaternion q(geom_quat.x, geom_quat.y, geom_quat.z, geom_quat.w);
            q.normalize();
            transform.setRotation(q);

            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", fixed_frame));

            // Update and Send Path
            detected_path.header = it.header;
            detected_path.header.frame_id = fixed_frame;
            geometry_msgs::PoseStamped pose;
            pose.header = it.header;
            pose.header.frame_id = fixed_frame;
            pose.pose.position.x= it.point.x;
            pose.pose.position.y = it.point.y;
            pose.pose.position.z = it.point.z;
            // pose.pose.orientation = geom_quat;
            detected_path.poses.push_back(pose);
        }
    }
        return;
}

void predicted_obj_cb(const traj_prediction::tracked_obj_arr msg){
    for (auto& it : msg.tracked_obj_arr) { 
            if (it.time_increment==0){
            visualization_msgs::Marker marker;
            // visualization_msgs::Marker marker;
            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            marker.header.frame_id = fixed_frame;
            marker.header.stamp = ros::Time::now();

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            marker.ns = "basic_shapes";
            marker.id = it.object_id;

            // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
            marker.type = shape;

            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            marker.action = visualization_msgs::Marker::ADD;

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = it.point.x;
            marker.pose.position.y = it.point.y;
            marker.pose.position.z = it.point.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration(0.1);
            marker_array.markers.push_back(marker);
            }


            if (it.object_id == 0 && it.time_increment==0.5){
                static tf::TransformBroadcaster br;
                tf::Transform transform;

                // Create Translation
                auto x = it.point.x;
                auto y = it.point.y;
                auto z = it.point.z;
                transform.setOrigin( tf::Vector3(x,y,z) );

                // Create Quaternion
                tf::Quaternion q(geom_quat.x, geom_quat.y, geom_quat.z, geom_quat.w);
                q.normalize();
                transform.setRotation(q);

                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", fixed_frame));

                // Update and Send Path
                predicted_path.header = it.header;
                predicted_path.header.frame_id = fixed_frame;
                geometry_msgs::PoseStamped pose;
                pose.header = it.header;
                pose.header.frame_id = fixed_frame;
                pose.pose.position.x= it.point.x;
                pose.pose.position.y = it.point.y;
                pose.pose.position.z = it.point.z;
                // pose.pose.orientation = geom_quat;
                predicted_path.poses.push_back(pose);
            }
    }
        return;
}



int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;
  ros::Publisher drone_traj_pub = node.advertise<nav_msgs::Path>("drone_traj", 10);
  ros::Publisher predicted_path_pub = node.advertise<nav_msgs::Path>("predicted_path", 10);
  ros::Publisher detected_path_pub = node.advertise<nav_msgs::Path>("detected_path", 10);
  ros::Publisher transformed_future_path_pub = node.advertise<nav_msgs::Path>("transformed_future_path", 10);
  ros::Publisher marker_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

  std::string position_topic, attitude_topic;

  std::string actual_pos_topic = "/tracked_obj_pos_arr";
  std::string predicted_pos_topic = "/predicted_obj_pos_arr2";
  std::string drone_attitude_topic = "/dji_sdk/attitude";
  std::string drone_pos_topic = "/dji_sdk/local_position";
  std::string planned_path_topic = "future_path";

  ros::Subscriber pos_sub = node.subscribe(actual_pos_topic, 10, &tracked_obj_cb);
  ros::Subscriber predicted_pos_sub = node.subscribe(predicted_pos_topic, 10, &predicted_obj_cb);
  ros::Subscriber att_sub = node.subscribe(drone_attitude_topic, 10, &attitude_cb);
  ros::Subscriber drone_pos_sub = node.subscribe(drone_pos_topic, 10, &drone_pos_cb);
  ros::Subscriber future_path_sub = node.subscribe(planned_path_topic,10,&future_path_cb);

  ros::Rate loop_rate(10);
//   while(detected_path.poses.size() < 1){
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
  while(ros::ok())
  {
    // if (predicted_path.poses.size() > 10){
    //     predicted_path.poses.clear();
    //     detected_path.poses.clear();
    // }
    predicted_path_pub.publish(predicted_path);
    detected_path_pub.publish(detected_path);
    drone_traj_pub.publish(drone_traj);
    transformed_future_path_pub.publish(transformed_future_path);
    marker_pub.publish(marker_array);
    marker_array.markers.clear();

    ros::spinOnce();
    loop_rate.sleep();
    
  }


  ros::spin();
  return 0;
};