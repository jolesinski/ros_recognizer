
#include <ros_recognizer/nodelet/viewer.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>

void ros_recognizer::Viewer::onInit()
{
  NODELET_INFO("Initializing viewer...");
  node_handle_ = getMTPrivateNodeHandle();
  visualizer_cfg_.reset(new DynamicConfigurator<Visualizer>("~/visualizer"));

  initTopics();

  auto sleep_time = 2*static_cast<float>(Visualizer::SPIN_MS)/1000;
  spinTimer = node_handle_.createTimer(ros::Duration(sleep_time), &Viewer::spinTimerCb, this);
}

void ros_recognizer::Viewer::spinTimerCb(const ros::TimerEvent& event)
{
  if(scene_subscriber_.hasNewData())
    visualizer_.setScene(scene_subscriber_.getDescription());
  if(model_subscriber_.hasNewData())
    visualizer_.setModel(model_subscriber_.getDescription());

  auto needs_redrawal = visualizer_cfg_->refresh(visualizer_);
  visualizer_.render(needs_redrawal);
}

void ros_recognizer::Viewer::initTopics()
{
  model_subscriber_.initTopics(node_handle_, "model");
  scene_subscriber_.initTopics(node_handle_, "scene");
}

PLUGINLIB_EXPORT_CLASS(ros_recognizer::Viewer, nodelet::Nodelet)