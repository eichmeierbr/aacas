#include <ros/ros.h>
#include <lidar_process/tracked_obj_arr.h>
// Cpp packages
#include <utility>      
#include<iostream> 
#include <cstdlib>
#include <vector>
#include <typeinfo> 
#include <unordered_map>
#include<cmath>
#include <fstream>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "geometry_msgs/Point.h"
using namespace std;



struct instance_pos{
    float x;
    float y;
    float z;
};

class traj_predictor{
    private:
    unordered_map<int, vector<pair<ros::Time, instance_pos>>>* obj_poses_dict; 
    ros::Subscriber tracked_obj_sub;
    public:
    traj_predictor(unordered_map<int, vector<pair<ros::Time, instance_pos>>>* obj_poses_dict){
        ros::NodeHandle n;
        this->obj_poses_dict = obj_poses_dict;
        tracked_obj_sub = n.subscribe ("/tracked_obj_pos_arr", 10, &traj_predictor::tracked_obj_cb,this);
    }

    void tracked_obj_cb(const lidar_process::tracked_obj_arr msg){
        // for ( const auto &x : obj_poses_dict )
        // {
        // std::cout << x.second.size()<< std::endl;
        // } 
        // cout << "here" << endl;
        for (auto& it : msg.tracked_obj_arr) {
            struct instance_pos obj;
                obj.x = it.point.x;
                obj.y = it.point.y;
                obj.z = it.point.z;
                if (!obj_poses_dict->count(it.object_id)){
                    vector<pair<ros::Time, instance_pos>> tmp_vec;
                    tmp_vec.push_back(make_pair(it.header.stamp,obj));
                    obj_poses_dict->insert({it.object_id, tmp_vec});
                }
                else{
                    (*obj_poses_dict)[it.object_id].push_back(make_pair(it.header.stamp,obj));
                }
        }
        return;
    }

};

void clear_obj_poses_dict(unordered_map<int, vector<pair<ros::Time, instance_pos>>> &obj_poses_dict){
    ros::Time curr_time = ros::Time::now();
    ros::Duration max_time_span(20);
    for (auto &x : obj_poses_dict )
        {
        for (auto it = begin(x.second); it!= end(x.second); ++it){
            cout << x.second.size() << endl;
            if ((curr_time - it->first) > max_time_span){
                x.second.erase(it);
            }
        } 
        } 

}

  
  int main (int argc, char** argv)
  {
    // Initialize ROS
    unordered_map<int, vector<pair<ros::Time, instance_pos>>> obj_poses_dict;
    ros::init (argc, argv, "lidar_process_node");
    traj_predictor traj_predictor(&obj_poses_dict); 
     ros::Rate loop_rate(10);
    while (ros::ok()){
        clear_obj_poses_dict(obj_poses_dict);
        // for ( const auto &x : obj_poses_dict )
        // {
        // std::cout << x.second.size()<< std::endl;
        // } 
        // cout << "blahh" << endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
  }
