//Author: Yanis DIALLO
//Date: 15/07/2023
//Contact: yanis.diallo.254@cranfield.ac.uk
//Objective: Convert Yolo bounding boxes to laser scan 

//C++
#include <math.h>
#include <iostream>

//ROS
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

//Vision
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/BoundingBox2D.h>
#include <geometry_msgs/Pose2D.h>

//LiDAR
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace cv;

float cam_x, cam_y;                   //Camera storing coordinates 
double fx = 530.4669406576809;  //Focal lenght
double fy = 530.4669406576809; 
double cx = 320.5;              //Principal points
double cy = 240.5;

ros::Publisher scan_pub; // Publisher LiDAR

void yoloCB(const vision_msgs::Detection2DArray::ConstPtr& detect_msg)
{

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  for (const auto& detection : detect_msg->detections)
  {
    //Center of the bounding box
    float px = detection.bbox.center.x;
    float py = detection.bbox.center.y;
    
    //Pixel to camera coordinates
    cam_x = (px - cx)/fx;
    cam_y = (py - cy)/fy;
        
    // Camera coordinates to LiDAR 
    try
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped = tfBuffer.lookupTransform("base_scan", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));

      // Define the point in the camera frame
      geometry_msgs::PointStamped cam_data;
      cam_data.header.frame_id = "camera_rgb_optical_frame";
      cam_data.point.x = cam_x;
      cam_data.point.y = cam_y;
      cam_data.point.z = 0.0;

      // Transform the point to the lidar frame
      geometry_msgs::PointStamped scan_data;
      tf2::doTransform(cam_data, scan_data, transformStamped);

      // Obtain the transformed coordinates
      float scan_x = scan_data.point.x;
      float scan_y = scan_data.point.y;
     
        // Lookup the transform from LiDAR frame to map frame
      geometry_msgs::TransformStamped lidar_to_map_transform;
      lidar_to_map_transform = tfBuffer.lookupTransform("map", "base_scan", ros::Time(0), ros::Duration(1.0));

      // Transform the point to the map frame
      geometry_msgs::PointStamped map_data;
      tf2::doTransform(scan_data, map_data, lidar_to_map_transform);

      // Obtain the transformed coordinates in the map frame
      float map_x = map_data.point.x;
      float map_y = map_data.point.y;
    
      //Laser scan publisher data
      sensor_msgs::LaserScan laser_scan;
      laser_scan.header.stamp = ros::Time::now();
      laser_scan.header.frame_id = "base_scan";
      laser_scan.angle_min = 0;
      laser_scan.angle_max = 6.28318977355957;
      laser_scan.angle_increment = 0.017501922324299812;
      laser_scan.time_increment = 0;
      laser_scan.scan_time = 0;
      laser_scan.range_min = 0.11999999731779099;
      laser_scan.range_max = 3.5; 
      laser_scan.ranges.push_back(sqrt(pow(scan_x, 2) + pow(scan_y, 2)));
      
      //if (laser_scan.range_min <= laser_scan.ranges[0] && laser_scan.ranges[0] <= laser_scan.range_max)
      //{
      scan_pub.publish(laser_scan);
      ROS_INFO("The center of the obstacle is %f m away", laser_scan.ranges[0]);
      cout << "-----------------------------------------------------------------------------------------------------------------------------------------" << endl;
      ROS_INFO("The center of the obstacle in map coordinates are x = %f, y = %f", map_x, map_y);
      //}
    
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      continue;
    }
  }
  //ROS_INFO("The center of the bounding box is x = %f, y = %f", cam_x, cam_y);
}


int main(int argc, char** argv)
{
  ros::init (argc, argv, "bounding_boxes_lidar_map");
  ros::NodeHandle nh;

  //Publisher LiDAR
  scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_yolo", 10);
  //Visualiser Subscriber
  ros::Subscriber yolo_sub = nh.subscribe<vision_msgs::Detection2DArray>("/yolov7/yolov7", 10, yoloCB);

  ros::spin();

  return 0;
  
}