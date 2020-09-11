#include <ros/ros.h>
#include <yolov3_sort/BoundingBox.h>
#include <yolov3_sort/BoundingBoxes.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>

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
#include <cstdlib>
#include <vector>
#include <typeinfo> 
#include <unordered_map>
#include<cmath>
#include <fstream>
#include <lidar_process/tracked_obj.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
using namespace std;


// pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

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
    private:
    float tx;
    float ty;
    float tz;
    float buffer;

    sensor_msgs::PointCloud2 pub_cloud;
    yolov3_sort::BoundingBox bb;
    yolov3_sort:: BoundingBoxes bbox_msg;
    sensor_msgs::PointCloud2ConstPtr point_cloud_msg;

    ros::Subscriber cloud_sub;
    ros::Subscriber bb_sub;
    ros::Publisher tracked_obj_pub;

    // Input Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
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

    void convert_to_pcl(const sensor_msgs::PointCloud2ConstPtr& msg){
        pcl::fromROSMsg (*msg, *input_cloud);
    }
    

    public:    
    pc_process():intrinsics(3,3),extrinsics(3,4),camera_matrix(3,4), 
    input_cloud(new pcl::PointCloud<pcl::PointXYZ>), 
    cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>), 
    proj_cloud_in_bb(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_in_bb(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>)
    {   
        ros::NodeHandle n;
        n.getParam("tx", tx);
        n.getParam("ty", ty);
        n.getParam("tz", tz);
        n.getParam("buffer", buffer);

        cloud_sub = n.subscribe ("/velodyne_points", 10, &pc_process::cloud_cb,this);
        bb_sub = n.subscribe ("/tracked_objects", 10, &pc_process::bb_cb, this);
        tracked_obj_pub = n.advertise<lidar_process::tracked_obj> ("tracked_obj_pos", 1);

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
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        tmp_cropped_cloud->is_dense = true;
        for (std::size_t i = 0; i < input_cloud->points.size(); ++i)
            {   
                //Flipping the lidar coordinates to adjust for the upside down velodyne installation
                float x = input_cloud->points[i].x;
                float y = input_cloud->points[i].y;
                float z = input_cloud->points[i].z;
                if ((x>0 && y>0 && x/y >1.5526) || (x>0 && y<0 && x/y <-1.5526)){    
                    pcl::PointXYZ point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    tmp_cropped_cloud->points.push_back(point);
                }
            }
            this->cropped_cloud = tmp_cropped_cloud;
        }

    bool get_points_in_bb(yolov3_sort::BoundingBox bb){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_in_bb(new pcl::PointCloud<pcl::PointXYZ>);
        tmp_cloud_in_bb ->is_dense = true;


        proj_cloud_in_bb->is_dense = true;
        cloud_in_bb-> is_dense = true;


        // viewer.showCloud(cropped_cloud);
        for (std::size_t i = 0; i < cropped_cloud->points.size (); ++i)
            {   
                // filter out the noise close to the lidar
                if (cropped_cloud->points[i].x > 0.01 && abs(cropped_cloud->points[i].y) > 0.01){
                    
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

            if (tmp_cloud_in_bb->points.size()>0){
                return true;
            }
            else{
                return false;
            }
        }
    
    
    void cluster(){

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

                if (cluster_indices.size() > 0){

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

                    //////////////////////////////////////VISUALIZATION
                    instance_pos*inst_pos_ptr = get_pos();
                    pcl::PointXYZ proj_point;
                    proj_point.x = inst_pos_ptr->x;
                    proj_point.y = inst_pos_ptr->y;
                    proj_point.z = 1;
                    tmp_cloud_cluster->points.push_back(proj_point);


                    // Adding grid points
                    pcl::PointCloud<pcl::PointXYZ>::Ptr grid_points(new pcl::PointCloud<pcl::PointXYZ>);
                    for (int i = -1; i < 2; i++){
                        for (int j=3; j < 6;j++){
                            pcl::PointXYZ point;
                            point.x = j;
                            point.y = i;
                            point.z = 1;
                            grid_points->points.push_back(point);
                        }
                    }

                    
                    // viewer->setBackgroundColor (0, 0, 0);
                    // //background points
                    // viewer->removePointCloud("all_points");
                    // viewer->addPointCloud<pcl::PointXYZ> (cropped_cloud, "all_points");

                    // //Grid Points
                    // viewer->removePointCloud("grid_points");
                    // pcl::visualization::PCLVisualizer::Ptr customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr grid_points);
                    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> grid_color (grid_points, 0, 255, 0);
                    // viewer->addPointCloud<pcl::PointXYZ> (grid_points, grid_color, "grid_points");
                    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "grid_points");



                    // viewer->removePointCloud("cloud_cluster");
                    // pcl::visualization::PCLVisualizer::Ptr customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr tmp_cloud_cluster);
                    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (tmp_cloud_cluster, 255, 0, 0);
                    // viewer->addPointCloud<pcl::PointXYZ> (tmp_cloud_cluster, single_color, "cloud_cluster");
                    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_cluster");
                    // // viewer->addCoordinateSystem (1.0);
                    // viewer->spinOnce (100);
                    //////////////////////////////////////////////////////////////////////

                    // // viewer.showCloud(tmp_cloud_cluster);
                }

    }
    
    float get_cluster_avg_detph( std::vector<pcl::PointIndices> cluster_indices, int idx){
        int size = cluster_indices[idx].indices.size();
        float total_depth = 0;
        float avg_depth = 0;
        for (std::vector<int>::const_iterator pit =  cluster_indices[idx].indices.begin (); pit != cluster_indices[idx].indices.end (); ++pit)
            {  
                total_depth += sqrt(pow(cloud_in_bb->points[*pit].x,2) + pow(cloud_in_bb->points[*pit].y,2)); 
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


    void publisher(int obj_indx , instance_pos*inst_pos_ptr){
        lidar_process::tracked_obj tracked_obj_msg;
        tracked_obj_msg.object_id = obj_indx; 
        tracked_obj_msg.pos_x = inst_pos_ptr ->x;
        tracked_obj_msg.pos_y = inst_pos_ptr ->y;
        tracked_obj_msg.pos_z = inst_pos_ptr ->z;
        tracked_obj_msg.header.stamp = ros::Time::now();
        tracked_obj_pub.publish(tracked_obj_msg);
    }

    void bb_cb(const yolov3_sort:: BoundingBoxes msg){
        bbox_msg = msg;
    
    } 


    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
        point_cloud_msg = msg;
        
        crop_cloud(point_cloud_msg);
        
        //loop through all bounding boxes
        for (int i =0; i<bbox_msg.bounding_boxes.size();i++){
            // instance ID of the object
            yolov3_sort::BoundingBox bb =  bbox_msg.bounding_boxes[i];
            int obj_indx = bb.idx;

            // instance already being tracked and tacklet died, delete the obj
            if (instance_pos_dict.count(obj_indx) && bb.label==-1){
                cout << "case 1 " << endl;
                cout << "deads tracklet obj_indx" << obj_indx<< endl;
                instance_pos*inst_pos_ptr = instance_pos_dict[obj_indx];
                instance_pos_dict.erase(obj_indx);
                delete inst_pos_ptr;
            }

            // instance already being tracked, tacklet still active
            else if (instance_pos_dict.count(obj_indx) && bb.label!=-1){
                // cout << "case 2 " << endl;
                if (get_points_in_bb(bb)==true)
                {
                    cluster();
                    //get position of the tracket 
                    instance_pos*inst_pos_ptr = get_pos();
                    instance_pos_dict[obj_indx] = inst_pos_ptr; 
                }
            }

            //instance not being tracked yet. 
            else if (!instance_pos_dict.count(obj_indx)){
                cout << "case 3" << endl;
                cout << " added new tracket " << obj_indx<< endl; 
		if (get_points_in_bb(bb)==true){
                    cluster();
                    //get position of the tracklet
                    instance_pos*inst_pos_ptr = get_pos();
                    instance_pos_dict.insert({obj_indx,inst_pos_ptr});
                }
            }
        }
	cout << instance_pos_dict.size() << endl;        
        for (auto const& x : instance_pos_dict)
        {   
            publisher(x.first , x.second);
        }

    }


};





  
  int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "lidar_process_node");
    pc_process pc_processer; 
    ros::spin ();
  }
