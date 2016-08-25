#include <ros_recognizer/nodelet/recognizer.h>

#include <pluginlib/class_list_macros.h>

void ros_recognizer::Recognizer::onInit()
{
  NODELET_INFO("Initializing recognizer...");
  scene_subscriber_ = node_handle_.subscribe("scene", 1, &Recognizer::sceneCallback, this);
  NODELET_INFO("Initialization done!");
}

void ros_recognizer::Recognizer::sceneCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  NODELET_INFO("Received scene cloud! Yippie!");
}

PLUGINLIB_EXPORT_CLASS(ros_recognizer::Recognizer, nodelet::Nodelet)