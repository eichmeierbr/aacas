#include <ros/ros.h>
#include <yolov3_sort/BoundingBox.h>
#include <yolov3_sort/BoundingBoxes.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// Cpp packages
#include<iostream> 
#include <vector>
#include <typeinfo> 
#include <unordered_map>
using namespace std;


sensor_msgs::PointCloud2 pub_cloud;
sensor_msgs::Image image_;
yolov3_sort::BoundingBox bb;
sensor_msgs::PointCloud2ConstPtr point_cloud_msg;

float tx;
float ty;
float tz;
float buffer;


pcl::visualization::CloudViewer viewer("PCL Viewer");

// Stores the estimated centoird location of the tracked object
struct instance_pos{
    float x;
    float y;
    float z;
    float avg;
    float std;
};


unordered_map<int ,instance_pos*> instance_pos_dict;

class pc_process{
    // Input Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    // Cropped Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud;
    //Projected Cloud in the bounding box
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud_in_bb;
    // 3D point of the corresponding points in bb 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_bb;
    // 3D point of the clustered cloud in bb
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster;
    
    
    
    Eigen:: MatrixXf intrinsics;
    Eigen:: MatrixXf extrinsics; 
    Eigen:: MatrixXf camera_matrix;
  

    public:    
    pc_process():intrinsics(3,3),extrinsics(3,4),camera_matrix(3,4), 
    cloud(new pcl::PointCloud<pcl::PointXYZ>), 
    cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>),
    proj_cloud_in_bb(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_in_bb(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>)
    {   
        
        // calibrated from matlab
        intrinsics << 574.0198, 0.0, 318.1983,
                    0.0, 575.2453, 246.5657, 
                    0.0, 0.0, 1.0;


        extrinsics << 1,0,0,tx,
                      0,1,0,ty,
                      0,0,1,tz;


        camera_matrix = intrinsics * extrinsics;
        
    }

    void crop_cloud(const sensor_msgs::PointCloud2ConstPtr& msg){
        convert_to_pcl(msg);
        cropped_cloud->is_dense = true;
        for (std::size_t i = 0; i < cloud->points.size(); ++i)
            {   
                //Flipping the lidar coordinates to adjust for the upside down velodyne installation
                float x = cloud->points[i].x;
                float y = cloud->points[i].y;
                float z = cloud->points[i].z;
                if ((x>0 && y>0 && x/y >1.5526) || (x>0 && y<0 && x/y <-1.5526)){    
                    pcl::PointXYZ point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    cropped_cloud->points.push_back(point);
                }
            }
            // viewer.showCloud(cropped_cloud);
            // pcl::toROSMsg(*cropped_cloud,pub_cloud);
        }

    bool get_points_in_bb(yolov3_sort::BoundingBox bb){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_in_bb(new pcl::PointCloud<pcl::PointXYZ>);
        tmp_cloud_in_bb ->is_dense = true;


        proj_cloud_in_bb->is_dense = true;
        cloud_in_bb-> is_dense = true;


        // viewer.showCloud(cropped_cloud);
        for (std::size_t i = 0; i < cropped_cloud->points.size (); ++i)
            {   
                // viewer.showCloud(cropped_cloud);
                // filter out the noise close to the lidar
                if (cropped_cloud->points[i].x > 0.01 && cropped_cloud->points[i].y > 0.01){
                    
                    // Project 3D points to 2D
                    Eigen::Vector4f three_loc;
                    three_loc <<  cropped_cloud->points[i].y, cropped_cloud->points[i].z,cropped_cloud->points[i].x, 1;
                    //  three_loc <<  cropped_cloud->points[i].y, cropped_cloud->points[i].z,cropped_cloud->points[i].x, 1;
                    Eigen::Vector3f two_loc;
                    two_loc = camera_matrix * three_loc;
                    
                     // Normalize. (These points are in camera coord)
                    float x = two_loc[0]/two_loc[2];
                    float y = two_loc[1]/two_loc[2];
                    float z = two_loc[2];

                    // int buffer =40;
                    //Select ones that are in the bounding box. 
                    if (x>bb.xmin-buffer && x<bb.xmax+buffer && y>bb.ymin-buffer && y<bb.ymax+buffer){
                        
                        //(These points are in camera coord)
                        pcl::PointXYZ point;
                        point.x = x;
                        point.y = y;
                        point.z = z;
                        proj_cloud_in_bb->points.push_back(point);

                        //Corresponding 3D points (These points are in lidar coord)
                        pcl::PointXYZ threeDpoint;
                        threeDpoint.x = cropped_cloud->points[i].x; 
                        threeDpoint.y = cropped_cloud->points[i].y; 
                        threeDpoint.z = cropped_cloud->points[i].z; 
                        tmp_cloud_in_bb->points.push_back(threeDpoint);

                    }
                }
            }
            this -> cloud_in_bb = tmp_cloud_in_bb;
            // viewer.showCloud(this->cloud_in_bb);
            if (tmp_cloud_in_bb->points.size()>0){
                return true;
            }
            else{
                return false;
            }
        // viewer.showCloud(cloud_in_bb);
        }
    
    
    void cluster(){
            // if (cloud_in_bb->points.size() > 0){
                // Create the segmentation object for the planar model and set all the parameters
                // pcl::SACSegmentation<pcl::PointXYZ> seg;

                // seg.setOptimizeCoefficients (true);
                // seg.setModelType (pcl::SACMODEL_PLANE);
                // seg.setMethodType (pcl::SAC_RANSAC);
                // seg.setMaxIterations (100);
                // seg.setDistanceThreshold (0.02);
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud (cloud_in_bb);
                        std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance (0.2); // 2cm
                ec.setMinClusterSize (5);
                ec.setMaxClusterSize (25000);
                ec.setSearchMethod (tree);
                ec.setInputCloud(cloud_in_bb);
                ec.extract (cluster_indices);


                int min_depth = 0;
                int min_index = 0;
                for (int i =0; i< cluster_indices.size();i++){
                    
                    float avg_depth = get_cluster_avg_detph(cluster_indices,i);
                    if (i==0) {
                        min_depth = avg_depth;
                        min_index =i;
                        }
                    else{
                        if (avg_depth < min_depth){
                            min_index =i;
                        }

                    }
                }

                for (std::vector<int>::const_iterator pit =  cluster_indices[min_index].indices.begin (); pit != cluster_indices[min_index].indices.end (); ++pit)
                    {
                        tmp_cloud_cluster->points.push_back (cloud_in_bb->points[*pit]); 
                    }
                
                this->cloud_cluster = tmp_cloud_cluster;
                viewer.showCloud(tmp_cloud_cluster);

    }
    
    float get_cluster_avg_detph( std::vector<pcl::PointIndices> cluster_indices, int idx){
        int size = cluster_indices[idx].indices.size();
        float total_depth = 0;
        float avg_depth = 0;
        for (std::vector<int>::const_iterator pit =  cluster_indices[idx].indices.begin (); pit != cluster_indices[idx].indices.end (); ++pit)
            {
                total_depth += cloud_in_bb->points[*pit].x; 
            }
        avg_depth = total_depth/size;
        return avg_depth;

    }





    instance_pos* get_pos(){
        instance_pos* inst_pos_ptr = new instance_pos();
        float x_avg = 0;
        float y_avg = 0;
        float z_avg = 0; 
        int count;

        for (std::size_t i = 0; i < cloud_cluster->points.size(); ++i)
        {   
            count = i+1; 
            x_avg = x_avg*(count-1)/count + cloud_cluster->points[i].x/count;
            y_avg = y_avg*(count-1)/count + cloud_cluster->points[i].y/count;
            z_avg = z_avg*(count-1)/count + cloud_cluster->points[i].z/count;
        }

        inst_pos_ptr->x = x_avg;
        inst_pos_ptr->y = y_avg;
        inst_pos_ptr->z = z_avg; 
        return inst_pos_ptr;
    }


    private:
    void convert_to_pcl(const sensor_msgs::PointCloud2ConstPtr& msg){
        pcl::fromROSMsg (*msg, *cloud);
    }

};



void bb_cb(const yolov3_sort:: BoundingBoxes msg){
    pc_process pc_processer; 
    pc_processer.crop_cloud(point_cloud_msg);
    //loop through all bounding boxes
    for (int i =0; i<msg.bounding_boxes.size();i++){
        // instance ID of the object
        yolov3_sort::BoundingBox bb =  msg.bounding_boxes[i];
        int obj_indx = bb.idx;
        // instance already being tracked, tacklet dead
        // if (bb.label == 0){
        if (instance_pos_dict.count(obj_indx) && bb.xmin==-1 && bb.xmax==-1){
            // cout << "case 1 " << endl;
            instance_pos*inst_pos_ptr = instance_pos_dict[obj_indx];
            instance_pos_dict.erase(obj_indx);
            delete inst_pos_ptr;
        }

        // instance already being tracked, tacklet still active
        else if (instance_pos_dict.count(obj_indx)){
            // cout << "case 2 " << endl;
            if (pc_processer.get_points_in_bb(bb)==true)
            {
                pc_processer.cluster();
                //get position of the tracket 
                instance_pos*inst_pos_ptr = pc_processer.get_pos();
                instance_pos_dict[obj_indx] = inst_pos_ptr; 
            }
        }

        else if (!instance_pos_dict.count(obj_indx)){
            // cout << "case 3" << endl;
            if (pc_processer.get_points_in_bb(bb)==true){
                pc_processer.cluster();
                //get position of the tracklet
                instance_pos*inst_pos_ptr = pc_processer.get_pos();
                instance_pos_dict.insert({obj_indx,inst_pos_ptr});
            }
        }
        // }
    }
    
    for (auto const& x : instance_pos_dict)
    {   
        cout << "Instance ID "<<x.first  // string (key)
                << ':' << endl;
        cout << "x" << x.second->x << endl;
        cout << "y" << x.second-> y<< endl;
    }

} 


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    point_cloud_msg = msg;
}



  
  int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "lidar_process_node");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("output", 1);

    n.getParam("tx", tx);
    n.getParam("ty", ty);
    n.getParam("tz", tz);
    n.getParam("buffer", buffer);

    
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber cloud_sub = n.subscribe ("/velodyne_points", 1, cloud_cb);
    ros::Subscriber bb_sub = n.subscribe ("/tracked_objects", 1, bb_cb);
    ros::Rate loop_rate(20);
    while (n.ok())
     {
    //    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
       pub.publish (pub_cloud);
       ros::spinOnce ();
       loop_rate.sleep ();
     }
    // Spin
    ros::spin ();
  }