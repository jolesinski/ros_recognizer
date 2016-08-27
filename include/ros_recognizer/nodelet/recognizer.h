
#ifndef ROS_RECOGNIZER_RECOGNIZER_H
#define ROS_RECOGNIZER_RECOGNIZER_H

#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <ros_recognizer/set_model_from_cloud.h>
#include <ros_recognizer/set_model_from_pcd.h>
#include <ros_recognizer/preprocessing/local_3d_describer.h>
#include <ros_recognizer/matching/local_matcher.h>
#include <ros_recognizer/verification/verifier.h>

namespace ros_recognizer
{

class Recognizer : public nodelet::Nodelet
{
public:
  //TODO: publisher for each step
  virtual void onInit();

  void sceneCallback(const sensor_msgs::PointCloud2ConstPtr& scene_msg);
  bool setModelFromCloud(set_model_from_cloud::Request& request,
                         set_model_from_cloud::Response& response);
  bool setModelFromPCD(set_model_from_pcd::Request& request,
                       set_model_from_pcd::Response& response);

private:
  ros::Subscriber scene_subscriber_;
  ros::ServiceServer model_cloud_service_;
  ros::ServiceServer model_pcd_service_;
  Local3dDescriber desciber;
  LocalMatcher matcher;
  Verifier verifier;

  Local3dDescription model_description;

  sensor_msgs::PointCloud2Ptr loadPCD(const std::string& pcd_path);
};

}


#endif //ROS_RECOGNIZER_RECOGNIZER_H
