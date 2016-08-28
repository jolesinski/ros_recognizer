
#include <ros_recognizer/nodelet/viewer.h>
#include <pluginlib/class_list_macros.h>

void ros_recognizer::Viewer::onInit()
{
  NODELET_INFO("Initializing viewer...");
  node_handle_ = getMTPrivateNodeHandle();
  visualizer_cfg_.reset(new DynamicConfigurator<Visualizer>("~/visualizer"));

  auto sleep_time = 3*static_cast<float>(Visualizer::SPIN_MS)/1000;
  spinTimer = node_handle_.createTimer(ros::Duration(sleep_time),
                                       &ros_recognizer::Viewer::spinTimerCb,
                                       this);
}

void ros_recognizer::Viewer::spinTimerCb(const ros::TimerEvent& event)
{
  NODELET_INFO("Updating visualizer...");
  visualizer_cfg_->refresh(visualizer_);
  visualizer_.render();
}

PLUGINLIB_EXPORT_CLASS(ros_recognizer::Viewer, nodelet::Nodelet)