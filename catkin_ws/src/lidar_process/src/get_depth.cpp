#include <ros/ros.h>
#include <yolov3_pytorch_ros/BoundingBox.h>
#include <yolov3_pytorch_ros/BoundingBoxes.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include<iostream> 
#include <vector>
using namespace std;


sensor_msgs::PointCloud2 pub_cloud;
sensor_msgs::Image image_;
yolov3_pytorch_ros::BoundingBox bb;

pcl::visualization::CloudViewer viewer("PCL Viewer");



class pc_process{
    // Input Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    // Cropped Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered;
    //Projected Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud;
    
    
    
    Eigen:: MatrixXf intrinsics;
    Eigen:: MatrixXf extrinsics; 
    Eigen:: MatrixXf camera_matrix;
  

    public:    
    pc_process():intrinsics(3,3),extrinsics(3,4),camera_matrix(3,4), 
    cloud(new pcl::PointCloud<pcl::PointXYZ>), 
    bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>),
    projected_cloud(new pcl::PointCloud<pcl::PointXYZ>)

    {   
        intrinsics << 612.8899536132812, 0.0, 313.19580078125,
                    0.0, 613.1091918945312, 248.6840362548828, 
                    0.0, 0.0, 1.0;

        extrinsics << 1,0,0,0,
                    0,1,0,0.005588,
                    0,0,1,0.012573;

        camera_matrix = intrinsics * extrinsics;
        
    }


    void crop_cloud(const sensor_msgs::PointCloud2ConstPtr& msg){
        convert_to_pcl(msg);
        bodyFiltered->is_dense = true;
        for (std::size_t i = 0; i < cloud->points.size(); ++i)
            {   
                //Flipping the lidar coordinates to adjust for the upside down velodyne installation
                float x = cloud->points[i].x;
                float y = -cloud->points[i].y;
                float z = -cloud->points[i].z;
                if ((x>0 && y>0 && x/y >1.5526) || (x>0 && y<0 && x/y <-1.5526)){    
                    pcl::PointXYZ point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    bodyFiltered->points.push_back(point);
                }
    
            }
        }

    void project_cloud(){

        //Project 3D pointcloud to 2D
        projected_cloud->is_dense = true;
        for (std::size_t i = 0; i < bodyFiltered->points.size (); ++i)
            {
                if (bodyFiltered->points[i].x > 0.01 && bodyFiltered->points[i].y > 0.01){
                    Eigen::Vector4f three_loc;
                    three_loc <<  bodyFiltered->points[i].y, bodyFiltered->points[i].z,bodyFiltered->points[i].x, 1;
                    Eigen::Vector3f two_loc;
                    two_loc = camera_matrix * three_loc;
                    
                    pcl::PointXYZ point;
                    point.x = two_loc[0]/two_loc[2];
                    point.y = two_loc[1]/two_loc[2];
                    point.z = two_loc[2];
                    // cout << "x:"<<point.x << endl;
                    // cout << "y:"<<point.y<< endl;
                    // cout <<"z:"<< point.z << endl;
                    projected_cloud->points.push_back(point);
                }
            }
        viewer.showCloud(projected_cloud );

        }

    private:
    void convert_to_pcl(const sensor_msgs::PointCloud2ConstPtr& msg){
        pcl::fromROSMsg (*msg, *cloud);
    }

};





void bb_cb(const yolov3_pytorch_ros:: BoundingBoxes msg){
    for (int i =0; i<msg.bounding_boxes.size();i++){
        if (msg.bounding_boxes[i].Class== "person"){
        bb= msg.bounding_boxes[i];
        }
    }
} 


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    pc_process pc_processer; 
    pc_processer.crop_cloud(msg);
    pc_processer.project_cloud();
}



  
//   void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
//   {
//     // Create a container for the data.
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered (new pcl::PointCloud<pcl::PointXYZ>);  
//     // pcl::PointCloud<pcl::PointXYZ> cloud;
//     pcl::fromROSMsg (*msg, *cloud);


//     // for(pcl::PointCloud<pcl::PointXYZ>::iterator it = *cloud.begin();it != *cloud.end();it ++) {
//     //     cout << "x: " << it-> x << endl; 
//     //     cout << "y: " <<it -> y << endl;
//     //     cout << "z: " <<it-> z << endl;  
//     // }
//     // Define min and max for X, Y and Z
//     float minX = 0, minY = -1.2, minZ = -2.5;
//     float maxX = +5, maxY = +1.2, maxZ = +2.5;


//     // Corp the pointcloud to a 70 degree FOV
//     bodyFiltered->is_dense = true;
//     for (std::size_t i = 0; i < cloud->points.size(); ++i)
//         {   
//             //Flipping the lidar coordinates to adjust for the upside down velodyne installation
//             float x = cloud->points[i].x;
//             float y = -cloud->points[i].y;
//             float z = -cloud->points[i].z;
//             if ((x>0 && y>0 && x/y >1.5526) || (x>0 && y<0 && x/y <-1.5526)){    
//                 pcl::PointXYZ point;
//                 point.x = x;
//                 point.y = y;
//                 point.z = z;
//                 bodyFiltered->points.push_back(point);
//             }
   
//         }

//     // viewer.showCloud(bodyFiltered);

//     Eigen:: MatrixXf intrinsics(3,3); 
//     Eigen:: MatrixXf extrinsics(3,4); 
//     Eigen:: MatrixXf camera_matrix(3,4); 
  

//     // intrinsics <<   384.46966552734375, 0, 314.06256103515625,
//     //                 0, 384.46966552734375, 248.62062072753906,
//     //                 0, 0, 1;

//     intrinsics << 612.8899536132812, 0.0, 313.19580078125,
//                  0.0, 613.1091918945312, 248.6840362548828, 
//                  0.0, 0.0, 1.0;

//     extrinsics << 1,0,0,0,
//                   0,1,0,0.005588,
//                   0,0,1,0.012573;



//     camera_matrix = intrinsics * extrinsics;

//     //Project 3D pointcloud to 2D
//     pcl::PointCloud<pcl::PointXYZ>::Ptr  projected_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//      projected_cloud->is_dense = true;
//     for (std::size_t i = 0; i < bodyFiltered->points.size (); ++i)
//         {
//             if (bodyFiltered->points[i].x > 0.01 && bodyFiltered->points[i].y > 0.01){
//                 Eigen::Vector4f three_loc;
//                 three_loc <<  bodyFiltered->points[i].y, bodyFiltered->points[i].z,bodyFiltered->points[i].x, 1;
//                 Eigen::Vector3f two_loc;
//                 two_loc = camera_matrix * three_loc;
                
//                 pcl::PointXYZ point;
//                 point.x = two_loc[0]/two_loc[2];
//                 point.y = two_loc[1]/two_loc[2];
//                 point.z = two_loc[2];
//                 // cout << "x:"<<point.x << endl;
//                 // cout << "y:"<<point.y<< endl;
//                 // cout <<"z:"<< point.z << endl;

//                  projected_cloud->points.push_back(point);
//             }

//         }
//     float total = 0;
//     int count = 0;
//     float avg = 0;
//     for (std::size_t i = 0; i <  projected_cloud->points.size (); ++i){
//         float x =    projected_cloud->points[i].x;
//         float y =    projected_cloud->points[i].y;
//         float z =    projected_cloud->points[i].z;

//         if (x>bb.xmin && x<bb.xmax && y>bb.ymin && y<bb.ymax){
//             total=total+z;
//             cout << z << endl;
//             count++;
//         }
//     }
//     // cout<<"total" << total << endl;
//     // cout << "count" << count << endl;
//     if (count != 0){
//         avg = total/count;
//         // cout << "Average" << avg << endl;
//     }
//     // avg = total/count;
//     // cout<< avg<< endl;
//     viewer.showCloud projected_cloud );


//     pcl::toROSMsg(   projected_cloud,pub_cloud);

//   } 
  
  
  int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "get_depth");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("output", 1);
    
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber cloud_sub = n.subscribe ("/velodyne_points", 1, cloud_cb);
    ros::Subscriber bb_sub = n.subscribe ("/detected_objects_in_image", 1, bb_cb);
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
