
#include <ros_recognizer/nodelet/viewer.h>
#include <pluginlib/class_list_macros.h>

void ros_recognizer::Viewer::onInit()
{
  NODELET_INFO("Initializing viewer...");
  node_handle_ = getMTPrivateNodeHandle();

  spinTimer = node_handle_.createTimer(ros::Duration(0.1),
                                       &ros_recognizer::Viewer::spinTimerCb,
                                       this);
}

void ros_recognizer::Viewer::spinTimerCb(const ros::TimerEvent& event)
{
  NODELET_INFO("Updating visualizer...");
}

PLUGINLIB_EXPORT_CLASS(ros_recognizer::Viewer, nodelet::Nodelet)