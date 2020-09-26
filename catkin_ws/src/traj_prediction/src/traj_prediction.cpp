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
    unordered_map<int, int> obj_labels;
    unordered_map<int, vector<pair<ros::Time, instance_pos>>>* obj_poses_dict; 
    ros::Subscriber tracked_obj_sub;
    public:
    traj_predictor(unordered_map<int, vector<pair<ros::Time, instance_pos>>>* obj_poses_dict){
        ros::NodeHandle n;
        this->obj_poses_dict = obj_poses_dict;
        tracked_obj_sub = n.subscribe ("/tracked_obj_pos_arr", 10, &traj_predictor::tracked_obj_cb,this);
    }

    void tracked_obj_cb(const lidar_process::tracked_obj_arr msg){
        for (auto& it : msg.tracked_obj_arr) {
            struct instance_pos obj;
                obj.x = it.point.x;
                obj.y = it.point.y;
                obj.z = it.point.z;
                // obj_labels.insert({it.object_id, it.object_type})
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
            cout << it->second.x << endl;
            // cout << x.second.size() << endl;
            // if the instance_pos entry is older than max_time_span, delete it
            if ((curr_time - it->first) > max_time_span){
                x.second.erase(it);
                //Delete the dictionary entry if the instance_pos vector is empty
                if (x.second.size() <1){
                    obj_poses_dict.erase(x.first);
                }
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
        ros::spinOnce();
        loop_rate.sleep();
    }
  }
