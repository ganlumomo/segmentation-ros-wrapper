#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

void pclConverterCallback(const sensor_msgs::Pointcloud2::ConstPtr& msg)
{
  sensor_msgs::Pointcloud out_pointcloud;
  sensor_msgs::convertPointCloud2ToPointCloud(msg, out_cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_converter");
  ros::NodeHandle nh;
  ros::Subscriber sub = n.subscribe("pclConverter", 10, pclConverterCallback);
  ros::spin();
  return 0;
}