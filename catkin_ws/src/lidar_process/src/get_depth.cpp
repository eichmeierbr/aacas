#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<iostream> 
using namespace std;

ros::Publisher pub;



// class lidar_process{
//     private: 
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered (new pcl::PointCloud<pcl::PointXYZ>);
//     ros::Subscriber sub
    
//     public: 
//     lidar_process(ros::NodeHandle *n){
//         sub = n->subscribe ("/velodyne_points", 1, cloud_cb);
//     }

//     void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg){
        
//     }


// }






//   pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Create a container for the data.
    // sensor_msgs::PointCloud2 output;

    // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 

    // pcl_conversions::toPCL(*msg, *cloud);




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
    float minX = -0.2, minY = -0.5, minZ = -2.5;
    float maxX = +0.2, maxY = +0.5, maxZ = +2.5;

    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*bodyFiltered);

    // viewer.addPointCloud (bodyFiltered,"body");
    // viewer.spin();


    for (std::size_t i = 0; i < bodyFiltered->points.size (); ++i)
    {
        cout << "x:"<<bodyFiltered->points[i].x << endl;
        // cout << "y:"<x<bodyFiltered->points[i].y << endl;
        // cout <<"z:"<< bodyFiltered->points[i].z << endl;
    }

    // Create a set of planar coefficients with X=Y=0,Z=1
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // coefficients->values.resize (4);
    // coefficients->values[0] = coefficients->values[1] = 0;
    // coefficients->values[2] = 1.0;
    // coefficients->values[3] = 0;

    // pcl::ProjectInliers<pcl::PointXYZ> proj;
    // proj.setModelType (pcl::SACMODEL_PLANE);
    // proj.setInputCloud (cloud);
    // proj.setModelCoefficients (coefficients);
    // proj.filter (*cloud_projected);
    
    // int count =0; 
    // double total=0; 
    // double avg;
    // for (std::size_t i = 0; i < cloud_projected->points.size (); ++i)
    // {
    //     // cout << "x:"<<cloud_projected->points[i].x << endl;
    //     // cout << "y:"<<cloud_projected->points[i].y << endl;
    //     // cout <<"z:"<< cloud_projected->points[i].z << endl;
    //     if( -0.2< cloud_projected->points[i].x && cloud_projected->points[i].x < 0.2&& -1< cloud_projected->points[i].y  && cloud_projected->points[i].y< 1){
    //         count++;
    //         total+= cloud->points[i].z;
    //         avg = total/count; 
    //         cout << avg << endl;
    //     }
    // }

        /*  METHOD #2: Using a Affine3f
        This method is easier and less error prone
    */
    float theta = M_PI/4;
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 2.5, 0.0, 0.0;

    // The same rotation matrix as before; theta radians around Z axis
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    // printf ("\nMethod #2: using an Affine3f\n");
    // std::cout << transform_2.matrix() << std::endl;





    // Do data processing here...
    // output = *input;
  
    // Publish the data.
    // pub.publish (output);
  } 
  
  
  int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "get_depth");
    ros::NodeHandle n;
  
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = n.subscribe ("/velodyne_points", 1, cloud_cb);
  
    // Create a ROS publisher for the output point cloud
    // pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  
    // Spin
    ros::spin ();
  }
