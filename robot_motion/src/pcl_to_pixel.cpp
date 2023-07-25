//Author: Yanis DIALLO
//Date: 15/07/2023
//Contact: yanis.diallo.254@cranfield.ac.uk
//Objective: Print point cloud data from a LiDAR on a image

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using namespace std;
using namespace cv;



//Global variables
tf2_ros::Buffer tf_buffer_;
boost::shared_ptr<sensor_msgs::Image> img_in;
vector<pcl::PointXYZ> coord_pcl;
float x, y, z; 
double u, v, w;
Mat img; 

vector<double> temp_pcl {0.0, 0.0, 0.0, 0.0};
vector<double> coord_img {0.0, 0.0, 0.0};
vector<vector<double>> P = {{530.4669406576809, 0.0, 320.5, -37.13268584603767},
                            {0.0, 530.4669406576809, 240.5, 0.0},
                            {0.0, 0.0, 1.0, 0.0}};
                        
ros::Publisher image_pub;


void imgCB(const sensor_msgs::Image::ConstPtr& img_viz)
{
    img_in = boost::make_shared<sensor_msgs::Image>(*img_viz);

     // Convert the received image message to a cv::Mat image
    cv_bridge::CvImagePtr cv_img;
    try
      {
       cv_img = cv_bridge::toCvCopy(img_viz, sensor_msgs::image_encodings::BGR8);
       }
    catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }
    
    img = cv_img->image;

    // Draw the pixel coordinates on the image for all LiDAR points
    for (const pcl::PointXYZ& point : coord_pcl)
    {
        x = point.x;
        y = point.y;
        z = point.z;
        temp_pcl = {x, y, z, 1};
        for (int i = 0; i < P.size(); i++)
        {
          double sum = 0.0;
          for (int j = 0; j < P[i].size(); j++)
          {
           sum += P[i][j] * temp_pcl[j];
          }
         coord_img[i] = sum;
        }
      u = coord_img[0]/coord_img[2];
      v = coord_img[1]/coord_img[2]; 
      ROS_INFO("The coordinates in pixels are : u = %f, v = %f ", u, v);
      cv::Point center(u, v);
      cv::circle(img, center, 3, cv::Scalar(0, 255, 0), -1);
    }

         // Publish the modified image
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        image_pub.publish(img_msg);
    
}


void pclCB(const sensor_msgs::PointCloud2::ConstPtr& pcl_in)
{
  
  try
    {
        // Transform the LiDAR point cloud to the camera frame
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_.lookupTransform("camera_rgb_optical_frame", pcl_in->header.frame_id, pcl_in->header.stamp, ros::Duration(1.0));

        //Convert PointCloud2 to PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_out(new pcl::PointCloud<pcl::PointXYZ>); //Variable for storing the conversion output
        pcl::fromROSMsg(*pcl_in, *pcl_out);

        // Transform each point in the LiDAR frame to the camera frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_to_cam(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_ros::transformPointCloud(*pcl_out, *pcl_to_cam, transform_stamped.transform);

        coord_pcl.clear();
   
      //Get the coordinates
      for(const pcl::PointXYZ& point : pcl_to_cam->points)
      {  
       coord_pcl.push_back(point);
      }
     
     if (img_in)
        {
            imgCB(img_in);
        }
        else
        {
            ROS_WARN("No image received yet");
        }

    } 

  catch (const tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
    }
}    

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pcl_to_image");
  ros::NodeHandle nh;

  // Create a tf2 buffer and listener
  tf2_ros::TransformListener tf_listener_(tf_buffer_, nh);


  //PointCloud Subscriber
  ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/laserPointCloud", 10, pclCB); //laserPointCloud

  //Visualiser Subscriber
  ros::Subscriber visu_sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 10, imgCB);

  // Create a publisher for the modified image
    image_pub = nh.advertise<sensor_msgs::Image>("/modified_image_topic", 10);

  ros::spin();

  return 0;
  
}