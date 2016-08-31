
#ifndef ROS_RECOGNIZER_RECOGNIZER_H
#define ROS_RECOGNIZER_RECOGNIZER_H

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <ros_recognizer/common/dynamic_configurator.h>
#include <ros_recognizer/conversions/description_ros.h>
#include <ros_recognizer/description/local_3d_describer.h>
#include <ros_recognizer/matching/local_matcher.h>
#include <ros_recognizer/set_model_from_cloud.h>
#include <ros_recognizer/set_model_from_pcd.h>
#include <ros_recognizer/verification/verifier.h>

namespace ros_recognizer
{

class Recognizer : public nodelet::Nodelet
{
public:
  virtual void onInit();

  void sceneCallback(const sensor_msgs::PointCloud2ConstPtr& scene_msg);
  bool setModelFromCloud(set_model_from_cloud::Request& request,
                         set_model_from_cloud::Response& response);
  bool setModelFromPCD(set_model_from_pcd::Request& request,
                       set_model_from_pcd::Response& response);

private:
  ros::NodeHandle node_handle_;

  // Input
  ros::Subscriber scene_subscriber_;
  ros::ServiceServer model_cloud_service_;
  ros::ServiceServer model_pcd_service_;

  // Output
  DescriptionPublisher scene_publisher_;
  DescriptionPublisher model_publisher_;
  ros::Publisher correspondences_publisher;
  ros::Publisher clusters_publisher;
  ros::Publisher valid_hyps_publisher;
  ros::Publisher false_hyps_publisher;

  Local3dDescriber desciber;
  std::unique_ptr<DynamicConfigurator<Local3dDescriber>> describer_cfg;

  LocalMatcher matcher;
  std::unique_ptr<DynamicConfigurator<LocalMatcher>> matcher_cfg;

  Verifier verifier;
  std::unique_ptr<DynamicConfigurator<Verifier>> verifier_cfg;

  Local3dDescription model_description;

  void initCfg();
  void initModel();
  void initTopics();

  sensor_msgs::PointCloud2Ptr loadPCD(const std::string& pcd_path);

  void refreshCfg();
  void publishResults(const Hypotheses& hyps);
};

}


#endif //ROS_RECOGNIZER_RECOGNIZER_H
