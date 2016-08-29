#include <ros_recognizer/nodelet/viewer.h>
#include <ros_recognizer/conversions/hypotheses_ros.h>
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

  valid_hyps_sub = node_handle_.subscribe("valid_hyps", 1, &Viewer::setValidHyps, this);
  false_hyps_sub = node_handle_.subscribe("false_hyps", 1, &Viewer::setFalseHyps, this);
}

void ros_recognizer::Viewer::setValidHyps(const geometry_msgs::PoseArrayPtr msg)
{
  Hypotheses hyps = convertMsgToHypotheses(*msg, model_subscriber_.getDescription().input_, true);
  NODELET_INFO("Received true hyps: %lu %lu", msg->poses.size(), hyps.size());
  visualizer_.setHypotheses(hyps, true);
}

void ros_recognizer::Viewer::setFalseHyps(const geometry_msgs::PoseArrayPtr msg)
{
  Hypotheses hyps = convertMsgToHypotheses(*msg, model_subscriber_.getDescription().input_, false);
  NODELET_INFO("Received false hyps: %lu %lu", msg->poses.size(), hyps.size());
  visualizer_.setHypotheses(hyps, false);
}

PLUGINLIB_EXPORT_CLASS(ros_recognizer::Viewer, nodelet::Nodelet)