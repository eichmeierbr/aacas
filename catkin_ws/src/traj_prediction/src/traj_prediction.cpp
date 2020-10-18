#include <ros/ros.h>
#include <lidar_process/tracked_obj.h>
#include <lidar_process/tracked_obj_arr.h>
// Cpp packages
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <utility>      
#include<iostream> 
#include <cstdlib>
#include <vector>
#include <typeinfo> 
#include <unordered_map>
#include<cmath>
#include <fstream>
#include "geometry_msgs/Point.h"
#include "Eigen/Dense"


using namespace std;

int pos_array_time_span; 

struct prediction_result{
    Eigen::MatrixXf prediction;
    ros::Time prediction_time;
};

struct instance_pos{
    float x;
    float y;
    float z;
};

class traj_predictor{
    private:
    //key: object_id  value:object_label
    unordered_map<int, int> *obj_labels;
    //key: object_id
    unordered_map<int, vector<pair<ros::Time, instance_pos>>>* obj_poses_dict; 
    ros::Subscriber tracked_obj_sub;
    ros::Publisher obj_trajectory_pub;
    // order of the polynomial used for prediction
    int pred_poly_order;
    //seconds into the future that we want to predict
    int secs_into_future = 2;
    //the number of interpolated time stamps between now and the last future time stamp for prediction
    int interlopated_pnts =4;
    // the number of position points availabel before the trajectory prediction is activated 
    int pred_pnt_thresh; 

    public:
    ros::Time curr_time;

    traj_predictor(unordered_map<int, vector<pair<ros::Time, instance_pos>>>* obj_poses_dict,
    unordered_map<int, int> *obj_labels){

        ros::NodeHandle n;
        n.getParam("pred_pnt_thresh", pred_pnt_thresh);
        n.getParam("pos_array_time_span", pos_array_time_span);
        n.getParam("pred_poly_order", pred_poly_order);
        this->obj_poses_dict = obj_poses_dict;
        this-> obj_labels = obj_labels;
        tracked_obj_sub = n.subscribe ("/tracked_obj_pos_arr", 10, &traj_predictor::tracked_obj_cb,this);
        obj_trajectory_pub = n.advertise<lidar_process::tracked_obj_arr> ("predicted_obj_pos_arr", 1);

    }

    void publisher( prediction_result* pred_result, int obj_id){
        lidar_process::tracked_obj_arr obj_pred_arr_msg;
        for (int i =0; i < pred_result -> prediction.rows(); i++){
            lidar_process::tracked_obj tracked_obj_msg;
            geometry_msgs::Point point;
            tracked_obj_msg.object_id = obj_id; 
            point.x = pred_result->prediction(i,0);
            point.y = pred_result->prediction(i,1);
            point.z = pred_result->prediction(i,2);
            tracked_obj_msg.point = point;
            tracked_obj_msg.header.stamp = pred_result->prediction_time;
            tracked_obj_msg.time_increment=  pred_result -> prediction(i,3);
            obj_pred_arr_msg.tracked_obj_arr.push_back(tracked_obj_msg);
        }
        obj_trajectory_pub.publish(obj_pred_arr_msg);
        delete pred_result;
    }

    void predict_traj(){
         for (auto &x : *obj_poses_dict){
             //only predict if there are at least 40 past data points
             if (x.second.size() > pred_pnt_thresh){
                    prediction_result* pred_result = poly_predict(pred_poly_order, x.first);
                    publisher(pred_result, x.first);
                // if ((*obj_labels)[x.first] == 0){
                //     // if (x.first==0){
                //     //     poly_predict(pred_poly_order, x.first);
                //     // }
                //     int count = 0;
                //     float x_avg =0;
                //     float y_avg = 0;
                //     float z_avg = 0;
                //     for (auto it = begin(x.second); it!= end(x.second); ++it){
                //     count ++;
                //     x_avg = x_avg*(count-1)/count + it -> second.x/count;
                //     y_avg = y_avg*(count-1)/count + it -> second.y/count;
                //     z_avg = z_avg*(count-1)/count + it -> second.z/count;
                //     }
                // }

             }
             
         }

    }


    //make prediction using a polynomial function
    prediction_result* poly_predict(int order, int obj_id){
        int col_num = order+1;
        int row_num = (*obj_poses_dict)[obj_id].size();
        Eigen::MatrixXf T(row_num, col_num);
        Eigen::VectorXf X(row_num);
        Eigen::VectorXf Y(row_num);
        Eigen::VectorXf Z(row_num);
        Eigen::MatrixXf x_beta(row_num, 1);
        Eigen::MatrixXf y_beta(row_num, 1);
        Eigen::MatrixXf z_beta(row_num, 1);
        ros::Time prediction_time =(*obj_poses_dict)[obj_id].back().first;
        // ros::Time prediction_time =ros::Time::now();
        for (int i = 0; i <(*obj_poses_dict)[obj_id].size(); i++){
            X(i) = (*obj_poses_dict)[obj_id][i].second.x;
            Y(i) = (*obj_poses_dict)[obj_id][i].second.y;
            Z(i) = (*obj_poses_dict)[obj_id][i].second.z;
            //constructing a matrix using all the past time stamps of the object
            for (int j=0; j <= order ;j++){
                double time_stamp = (*obj_poses_dict)[obj_id][i].first.toSec();
                T(i,j) = pow(time_stamp-prediction_time.toSec(), j);
            }
        }

        // Finding the polynomial parameters between T and X Y Z
        x_beta = (T.transpose()*T).inverse()*T.transpose()*X;
        y_beta = (T.transpose()*T).inverse()*T.transpose()*Y;
        z_beta = (T.transpose()*T).inverse()*T.transpose()*Z;

        //make the prediction
        Eigen::MatrixXf future_T(interlopated_pnts, col_num);
        Eigen::MatrixXf T_pred(interlopated_pnts,1);

        for (int i =0 ; i < interlopated_pnts; i++){
            float time_stamp = (float)(secs_into_future/(float)interlopated_pnts)*(i+1); 
            T_pred(i,0) = time_stamp;
            for (int j=0; j <= order ;j++){
                future_T(i,j) = pow(time_stamp, j);
            }
        }

        Eigen::MatrixXf x_pred = future_T * x_beta;
        Eigen::MatrixXf y_pred = future_T * y_beta;
        Eigen::MatrixXf z_pred = future_T * z_beta;

        Eigen::MatrixXf pred(x_pred.rows(), x_pred.cols() + y_pred.cols() + z_pred.cols()+T_pred.cols());
        pred <<  x_pred,  y_pred,  z_pred , T_pred;

        prediction_result*pred_result = new prediction_result();
        pred_result-> prediction = pred;
        pred_result->prediction_time = prediction_time;

        return pred_result;
    }


    void tracked_obj_cb(const lidar_process::tracked_obj_arr msg){
        for (auto& it : msg.tracked_obj_arr) {
            struct instance_pos obj;
                obj.x = it.point.x;
                obj.y = it.point.y;
                obj.z = it.point.z;
                obj_labels->insert({it.object_id, it.object_label});
                this->curr_time = it.header.stamp;
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


void clear_obj_poses_dict(unordered_map<int, vector<pair<ros::Time, instance_pos>>> &obj_poses_dict,
unordered_map<int, int> &obj_labels, traj_predictor* traj_predictor){
    // ros::Time curr_time = ros::Time::now();
    // Keeping the recent 5 seconds worth of data
    ros::Duration max_time_span(pos_array_time_span);
    vector<int> dead_keys; 
    for (auto &x : obj_poses_dict )
        {
        // cout << "object ids " << x.first<< endl;
        // cout << "size" << x.second.size() << endl;
        // cout << "    " << endl;
        for (auto it = begin(x.second); it!= end(x.second);){
            // if the instance_pos entry is older than max_time_span, delete it
            if ((traj_predictor->curr_time - it->first) > max_time_span){
                x.second.erase(it);
                // Delete the dictionary entry if the instance_pos vector is empty
                if (x.second.size() < 1){
                    dead_keys.push_back(x.first);
                }
            }
            else{
                ++it;
            }
        } 
    } 

    // Delete the keys that have empty values in the dictionaries
    for (int i=0; i < dead_keys.size(); i++){
        obj_poses_dict.erase(dead_keys[i]);
        obj_labels.erase(dead_keys[i]);
    }

}

  
  int main (int argc, char** argv)
  {
    // Initialize ROS
    unordered_map<int, vector<pair<ros::Time, instance_pos>>> obj_poses_dict;
    unordered_map<int, int> obj_labels;
    ros::init (argc, argv, "lidar_process_node");
    traj_predictor traj_predictor(&obj_poses_dict, &obj_labels); 
    ros::Rate loop_rate(20);
    while (ros::ok()){
        clear_obj_poses_dict(obj_poses_dict, obj_labels, &traj_predictor);
        traj_predictor.predict_traj();
        ros::spinOnce();
        loop_rate.sleep();
    }
  }
