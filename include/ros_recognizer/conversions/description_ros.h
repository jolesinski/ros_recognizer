
#ifndef ROS_RECOGNIZER_DESCRIPTION_TO_ROS_H
#define ROS_RECOGNIZER_DESCRIPTION_TO_ROS_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <ros_recognizer/preprocessing/local_3d_describer.h>

namespace ros_recognizer
{

// Temp classes to see if custom msgs are necessary
class DescriptionPublisher
{
public:
  void initTopics(ros::NodeHandle& nh, const std::string& topic_prefix)
  {
    input_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(topic_prefix + "_input", 1);
    normals_publisher_ = nh.advertise<pcl::PointCloud<pcl::Normal>>(topic_prefix + "_normals", 1);
    keypoints_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(topic_prefix + "_keypoints", 1);
  };

  void publish(const Local3dDescription& descr)
  {
    input_publisher_.publish(descr.input_);
    normals_publisher_.publish(descr.normals_);
    keypoints_publisher_.publish(descr.keypoints_);
  }

private:
  ros::Publisher input_publisher_;
  ros::Publisher normals_publisher_;
  ros::Publisher keypoints_publisher_;
};

class DescriptionSubscriber
{
public:
  void initTopics(ros::NodeHandle& nh, const std::string& topic_prefix)
  {
    cloud_subscriber_ = nh.subscribe(topic_prefix + "_input", 1, &DescriptionSubscriber::setCloud, this);
    normals_subscriber_ = nh.subscribe(topic_prefix + "_normals", 1, &DescriptionSubscriber::setNormals, this);
    keypoints_subscriber_ = nh.subscribe(topic_prefix + "_keypoints", 1, &DescriptionSubscriber::setKeypoints, this);
  };

  Local3dDescription getDescription()
  {
    has_new_data_ = false;
    return description_;
  }

  bool hasNewData()
  {
    return has_new_data_;
  }

private:
  Local3dDescription description_;
  bool has_new_data_ = false;

  ros::Subscriber cloud_subscriber_;
  ros::Subscriber normals_subscriber_;
  ros::Subscriber keypoints_subscriber_;

  void setCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr msg)
  {
    has_new_data_ = true;
    description_.input_ = msg;
  }

  void setNormals(const pcl::PointCloud<pcl::Normal>::Ptr msg)
  {
    has_new_data_ = true;
    description_.normals_ = msg;
  }

  void setKeypoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr msg)
  {
    has_new_data_ = true;
    description_.keypoints_ = msg;
  }
};

}

#endif //ROS_RECOGNIZER_DESCRIPTION_TO_ROS_H
