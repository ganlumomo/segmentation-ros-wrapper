#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// void pclConverterCallback(const sensor_msgs::Pointcloud2::ConstPtr& msg)
// {
//   sensor_msgs::Pointcloud out_pointcloud;
//   sensor_msgs::convertPointCloud2ToPointCloud(msg, out_cloud);
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "pcl_converter");
//   ros::NodeHandle nh;
//   ros::Subscriber sub = n.subscribe("pclConverter", 10, pclConverterCallback);
//   ros::Publisher
//   ros::spin();
//   return 0;
// }


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::PointCloud>("/labeled_pointcloud", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/labeled_pcl2", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    sensor_msgs::PointCloud out_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(msg, out_cloud);
    //.... do something with the input and generate the output...
    pub_.publish(out_cloud);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}