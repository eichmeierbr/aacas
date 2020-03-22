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
//         boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
//         float maxX = +0.2, maxY = +0.5, maxZ = +2.5;
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


sensor_msgs::PointCloud2 pub_cloud;
sensor_msgs::Image image_;
// pcl::PointCloud<pcl::PointXYZ>::Ptr pub_msg;


//  pcl::visualization::CloudViewer viewer("PCL Viewer");
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
    float minX = 0, minY = -5, minZ = -2.5;
    float maxX = +5, maxY = +5, maxZ = +2.5;

    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*bodyFiltered);
    //     for (std::size_t i = 0; i < bodyFiltered->points.size (); ++i)
    // {
    //     cout << "x:"<<bodyFiltered->points[i].x << endl;
    //     cout << "y:"<<bodyFiltered->points[i].y << endl;
    //     cout <<"z:"<< bodyFiltered->points[i].z << endl;
    // }
    // viewer.showCloud (bodyFiltered);

    // int image_size_x = 640, image_size_y = 480;
    // float center_x = (640.0f / 2.0f), center_y = (480.0f / 2.0f);
    // float focal_length_x = 525.0f, focal_length_y = focal_length_x;
    // Eigen::Affine3f sensor_pose = Eigen::Affine3f(Eigen::Translation3f(bodyFiltered->sensor_origin_[0],
    //             bodyFiltered->sensor_origin_[1],
    //             bodyFiltered->sensor_origin_[2])) *
    //             Eigen::Affine3f(bodyFiltered->sensor_orientation_);
    // float noise_level = 0.0f, minimum_range = 0.0f;

    // pcl::RangeImagePlanar::Ptr range_image_ptr(new pcl::RangeImagePlanar);
    // pcl::RangeImagePlanar& range_image = *range_image_ptr; 

    // // pcl::RangeImagePlanar range_image;
    // range_image.createFromPointCloudWithFixedSize(*bodyFiltered, image_size_x, image_size_y,
    //     center_x, center_y, focal_length_x, focal_length_x,
    //     sensor_pose, pcl::RangeImage::CAMERA_FRAME,
    //     noise_level, minimum_range);


    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    // float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
    // float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    // float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    // Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    // pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    // float noiseLevel=0.00;
    // float minRange = 0.0f;
    // int borderSize = 1;

    // pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
    // pcl::RangeImage& rangeImage = *range_image_ptr; 
    
    // // pcl::RangeImage rangeImage;
    // rangeImage.createFromPointCloud(*bodyFiltered, angularResolution, maxAngleWidth, maxAngleHeight,
    //                                 sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    // cout<< rangeImage << endl;
    // for (int i =0;i< rangeImage.points.size();i++){
    //     cout <<rangeImage.points[i] << endl;
    // }
    // viewer3D.setBackgroundColor (1, 1, 1);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
    // viewer3D.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
    // viewer3D.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
    // viewer3D.initCameraParameters ();
    // setViewerPose(viewer3D, rangeImage.getTransformationToWorldSystem ());


        /*  METHOD #2: Using a Affine3f
        This method is easier and less error prone
    */
    float theta = M_PI/4;
    // Eigen::Affine3f extrinsics = Eigen::Affine3f::Identity();
    // Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Identity();
    Eigen:: MatrixXf projection_matrix(3,4); 
    // projection_matrix <<384.46966552734375, 0.0, 314.06256103515625, 0.0, 
    //             0.0, 384.46966552734375, 248.62062072753906, 0.0, 
    //             0.0, 0.0, 1.0, 0.0,
    //             0,0,0,1;

    // projection_matrix << 612.8899536132812, 0.0, 313.19580078125, 0.0, 
    //                     0.0, 613.1091918945312, 248.6840362548828, 0.0, 
    //                     0.0, 0.0, 1.0, 0.0;
    

    // Eigen::Matrix4f intrinsics = Eigen::Matrix4f::Identity();  

    projection_matrix <<   384.46966552734375, 0, 314.06256103515625, 0,
                    0, 384.46966552734375, 248.62062072753906,0,
                    0, 0, 1, 0;

    // Eigen::Matrix4f camerat_matrix = intrinsics * extrinsics;
    






    // Define a translation of 2.5 meters on the x axis.
    // extrinsics.translation() << 2.5, 0.0, 5;

    // // The same rotation matrix as before; theta ` 56radians around Z axis
    // extrinsics.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    // printf ("\nMethod #2: using an Affine3f\n");
    // std::cout << extrinsics.matrix() << std::endl;
      // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    for (std::size_t i = 0; i < bodyFiltered->points.size (); ++i)
        {
            if (bodyFiltered->points[i].x > 0.01 && bodyFiltered->points[i].y > 0.01){
                Eigen::Vector4f three_loc;
                three_loc <<  bodyFiltered->points[i].y, bodyFiltered->points[i].z,bodyFiltered->points[i].x, 1;
                Eigen::Vector3f two_loc;
                two_loc = projection_matrix * three_loc;
                
                pcl::PointXYZ point;
                point.x = two_loc[0]/two_loc[2]/1000;
                point.y = two_loc[1]/two_loc[2]/1000;
                point.z = two_loc[2]/1000;
                // cout << "x:"<<point.x+320 << endl;
                // cout << "y:"<<point.y +240<< endl;
                // cout <<"z:"<< point.z << endl;

                transformed_cloud->points.push_back(point);
            }

        }
    // You can either apply transform_1 or extrinsics; they are the same
    // pcl::transformPointCloud (*bodyFiltered, *transformed_cloud, projection_matrix);

    // for (std::size_t i = 0; i < transformed_cloud->points.size (); ++i)
    // {
    //     cout << "x:"<<transformed_cloud->points[i].x << endl;
    //     cout << "y:"<<transformed_cloud->points[i].y << endl;
    //     cout <<"z:"<< transformed_cloud->points[i].z << endl;
    // }

    pcl::toROSMsg(*transformed_cloud,pub_cloud);
    // pcl::toROSMsg (*transformed_cloud, image_);

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
