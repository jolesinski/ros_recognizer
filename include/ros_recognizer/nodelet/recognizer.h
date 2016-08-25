
#ifndef ROS_RECOGNIZER_RECOGNIZER_H
#define ROS_RECOGNIZER_RECOGNIZER_H

#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

namespace ros_recognizer
{

class Recognizer : public nodelet::Nodelet
{
public:
  //TODO: publisher for each step
  virtual void onInit();

  void sceneCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber scene_subscriber_;
};

}


#endif //ROS_RECOGNIZER_RECOGNIZER_H
