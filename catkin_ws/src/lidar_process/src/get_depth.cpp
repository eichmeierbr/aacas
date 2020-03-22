#include <ros/ros.h>
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
using namespace std;


sensor_msgs::PointCloud2 pub_cloud;
sensor_msgs::Image image_;
// pcl::PointCloud<pcl::PointXYZ>::Ptr pub_msg;


 pcl::visualization::CloudViewer viewer("PCL Viewer");
//  pcl::visualization::PCLVisualizer viewer3D ("3D Viewer");
  
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Create a container for the data.


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered (new pcl::PointCloud<pcl::PointXYZ>);  
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*msg, *cloud);


    // for(pcl::PointCloud<pcl::PointXYZ>::iterator it = *cloud.begin();it != *cloud.end();it ++) {
    //     cout << "x: " << it-> x << endl; 
    //     cout << "y: " <<it -> y << endl;
    //     cout << "z: " <<it-> z << endl;  
    // }
    // Define min and max for X, Y and Z
    float minX = 0, minY = -1.2, minZ = -2.5;
    float maxX = +5, maxY = +1.2, maxZ = +2.5;

    // Eigen::Vector3f boxRotation;
    // boxRotation[0]=0;  // rotation around x-axis
    // boxRotation[1]=0;  // rotation around y-axis
    // boxRotation[2]=0.785;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.


    // pcl::CropBox<pcl::PointXYZ> boxFilter;
    // boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    // boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    // boxFilter.setInputCloud(cloud);
    // boxFilter.filter(*bodyFiltered);

    // Corp the pointcloud to a 70 degree FOV
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

    // viewer.showCloud(bodyFiltered);

    Eigen:: MatrixXf intrinsics(3,3); 
    Eigen:: MatrixXf extrinsics(3,4); 
    Eigen:: MatrixXf camera_matrix(3,4); 
  

    intrinsics <<   384.46966552734375, 0, 314.06256103515625,
                    0, 384.46966552734375, 248.62062072753906,
                    0, 0, 1;

    extrinsics << 1,0,0,0,
                  0,1,0,0,
                  0,0,1,0.0045;



    camera_matrix = intrinsics * extrinsics;

    //Project 3D pointcloud to 2D
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    transformed_cloud->is_dense = true;
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
                cout << "x:"<<point.x << endl;
                cout << "y:"<<point.y<< endl;
                cout <<"z:"<< point.z << endl;

                transformed_cloud->points.push_back(point);
            }

        }
         viewer.showCloud(transformed_cloud );


    pcl::toROSMsg(*transformed_cloud,pub_cloud);

  } 
  
  
  int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "get_depth");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("output", 1);
    
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = n.subscribe ("/velodyne_points", 1, cloud_cb);

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
