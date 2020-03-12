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
 #include <pcl/visualization/cloud_viewer.h>
#include<iostream> 
using namespace std;

// ros::Publisher pub;



// class lidar_process{ 

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered;
//     pcl::CropBox<pcl::PointXYZ> boxFilter;
//     ros::Subscriber sub;
    
//     public: 
//     lidar_process(ros::NodeHandle *n){  
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered (new pcl::PointCloud<pcl::PointXYZ>);
//         // pcl::CropBox<pcl::PointXYZ> boxFilter;
//         sub = n->subscribe ("/velodyne_points", 1, &lidar_process::cloud_cb,this);
//     }

//     void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg){
//         pcl::fromROSMsg (*msg, *cloud);
//         crop_point_cloud(); 
//         for (std::size_t i = 0; i < bodyFiltered->points.size (); ++i)
//         {
//             cout << "x:"<<bodyFiltered->points[i].x << endl;
//             // cout << "y:"<x<bodyFiltered->points[i].y << endl;
//             // cout <<"z:"<< bodyFiltered->points[i].z << endl;
//         }

//     }

//     void crop_point_cloud(){
//         float minX = -0.2, minY = -0.5, minZ = -2.5;
//         float maxX = +0.2, maxY = +0.5, maxZ = +2.5;
//         boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
//         boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
//         boxFilter.setInputCloud(cloud);
//         boxFilter.filter(*bodyFiltered);

//     }


// };

//   int main (int argc, char** argv)
//   {
//     // Initialize ROS
//     ros::init (argc, argv, "get_depth");
//     ros::NodeHandle n;
//     lidar_process lidar_process_node(&n);
  
//     // Spin
//     ros::spin ();
//     return 0;
//   }


// pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered;


sensor_msgs::PointCloud2 pub_cloud;

// pcl::PointCloud<pcl::PointXYZ>::Ptr pub_msg;


//   pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  
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
    float minX = 0, minY = -5, minZ = -2.5;
    float maxX = +5, maxY = +5, maxZ = +2.5;

    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*bodyFiltered);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr pub_msg (new pcl::PointCloud<pcl::PointXYZ>);  
    // msg->header.frame_id = "some_tf_frame";
    // msg->height = msg->width = 1;
    // msg->points.push_back(*bodyFiltered);

    pcl::toROSMsg(*bodyFiltered,pub_cloud);
    // pub_msg = bodyFiltered;

    // viewer.addPointCloud (bodyFiltered,"body");
    // viewer.spin();

    // pcl::visualization::PCLVisualizer viewer("Cloud Viewer");


    for (std::size_t i = 0; i < bodyFiltered->points.size (); ++i)
    {
        cout << "x:"<<bodyFiltered->points[i].x << endl;
        // cout << "y:"<x<bodyFiltered->points[i].y << endl;
        // cout <<"z:"<< bodyFiltered->points[i].z << endl;
    }


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


  } 
  
  
  int main (int argc, char** argv)
  {
    // Initialize ROS
    ros::init (argc, argv, "get_depth");
    ros::NodeHandle n;

    // Create a ROS publisher for the output point cloud
    // ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("output", 1);
    // ros::Publisher pub = n.advertise<pcl::PointCloud<pcl::PointXYZ>> ("filtered_lidar_output", 1);
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
