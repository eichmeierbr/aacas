#include <lidar_process/tracked_obj_arr.h>

// Cpp packages
#include<iostream> 
#include <cstdlib>
#include <vector>
#include <typeinfo> 
#include <unordered_map>
#include<cmath>
#include <fstream>
#include <rviz_visual_tools/rviz_visual_tools.h>
using namespace std;



class traj_predictor{
    private:
    ros::Subscriber tracked_obj_sub;
    
    public:
    traj_predictor(){
        ros::NodeHandle n;
        tracked_obj_sub = n.subscribe ("/tracked_obj_pos_arr", 10, &traj_predictor::tracked_obj_cb,this);
    }

    void tracked_obj_cb(const lidar_process::tracked_obj_arr msg){
        for (auto& it : msg.tracked_obj_arr) {
            // cout << it->x << endl;
            cout << "object id"<< it.object_id << endl;
            cout << it.pos_x << endl;
        }
        return;
    }

};



  
  int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "lidar_process_node");
    traj_predictor traj_predictor; 
    ros::spin ();
  }
