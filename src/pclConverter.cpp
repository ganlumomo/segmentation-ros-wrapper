#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

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
    sub_ = n_.subscribe("/labeled_array", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const std_msgs::Float32MultiArrayConstPtr& msg)
  {
    sensor_msgs::PointCloud out_cloud;
    out_cloud.header.frame_id = "camera_color_optical_frame";
    std::vector<float> points_data = msg->data;
    int h = msg->layout.dim[0].size;
    int w = msg->layout.dim[1].size;
    int stride = msg->layout.dim[1].stride;
    geometry_msgs::Point32 p;
    std::vector<ChannelFloat32> channels;
    sensor_msgs::ChannelFloat32 channel;
    for (int i = 0; i < h; ++i)
    {
      p.x = points_data[0 + i*stride];
      p.y = points_data[1 + i*stride];
      p.z = points_data[2 + i*stride];
      out_cloud.points.push_back(p);
      channel.values.push_back(points_data[3 + i*stride]);
    }
    channels.push_back(channel);
    out_cloud.channels = channels;
    // sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_cloud);
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
