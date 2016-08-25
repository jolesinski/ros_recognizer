#include <ros_recognizer/nodelet/recognizer.h>

#include <pluginlib/class_list_macros.h>

void ros_recognizer::Recognizer::onInit()
{
  NODELET_INFO("Initializing recognizer...");
  scene_subscriber_ = node_handle_.subscribe("scene", 1, &Recognizer::sceneCallback, this);
  model_cloud_service_ = node_handle_.advertiseService("set_model_from_cloud",
                                                       &Recognizer::setModelFromCloud, this);
  model_pcd_service_ = node_handle_.advertiseService("set_model_from_pcd",
                                                     &Recognizer::setModelFromPCD, this);
  NODELET_INFO("Initialization done!");
}

void ros_recognizer::Recognizer::sceneCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  NODELET_INFO("Received scene cloud! Yippie!");
}

bool ros_recognizer::Recognizer::setModelFromCloud(ros_recognizer::set_model_from_cloud::Request& request,
                                                   ros_recognizer::set_model_from_cloud::Response& response)
{
  NODELET_INFO("Received new model cloud! Yayey!");
  return true;
}

bool ros_recognizer::Recognizer::setModelFromPCD(ros_recognizer::set_model_from_pcd::Request& request,
                                                 ros_recognizer::set_model_from_pcd::Response& response)
{
  NODELET_INFO("Received new model pcd! Yahoo!");
  return true;
}

PLUGINLIB_EXPORT_CLASS(ros_recognizer::Recognizer, nodelet::Nodelet)