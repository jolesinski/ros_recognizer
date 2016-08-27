#include <ros_recognizer/nodelet/recognizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>

void ros_recognizer::Recognizer::onInit()
{
  NODELET_INFO("Initializing recognizer...");
  node_handle_ = getMTPrivateNodeHandle();

  initCfg();
  initModel();
  initTopics();

  NODELET_INFO("Initialization done!");
}

void ros_recognizer::Recognizer::sceneCallback(const sensor_msgs::PointCloud2ConstPtr& scene_msg)
{
  NODELET_INFO("Received scene cloud! Yippie!");
  refreshCfg();

  auto scene_description = desciber(*scene_msg);
  auto hypotheses = matcher(model_description, scene_description);
  hypotheses = verifier(hypotheses, scene_description.input_);
  auto instances = std::count_if(std::begin(hypotheses), std::end(hypotheses),
                                [](const Hypothesis& hyp) { return hyp.is_valid_; });
  NODELET_INFO("Found %ld instances out of %lu hypotheses", instances, hypotheses.size());
}

bool ros_recognizer::Recognizer::setModelFromCloud(ros_recognizer::set_model_from_cloud::Request& request,
                                                   ros_recognizer::set_model_from_cloud::Response& response)
{
  NODELET_INFO("Received new model cloud! Yayey!");
  refreshCfg();

  model_description = desciber(request.model_cloud);

  return true;
}

bool ros_recognizer::Recognizer::setModelFromPCD(ros_recognizer::set_model_from_pcd::Request& request,
                                                 ros_recognizer::set_model_from_pcd::Response& response)
{
  //TODO: add rostest
  NODELET_INFO("Received new model pcd path! Yahoo!");
  refreshCfg();

  sensor_msgs::PointCloud2Ptr model_cloud;
  try
  {
    model_cloud = loadPCD(request.pcd_path);
  }
  catch (const std::runtime_error& err)
  {
    NODELET_ERROR("Error while setting model: %s", err.what());
    return false;
  }
  model_description = desciber(*model_cloud);

  NODELET_INFO("Loaded model from pcd! Yahoo!");
  return true;
}

sensor_msgs::PointCloud2Ptr ros_recognizer::Recognizer::loadPCD(const std::string& pcd_path)
{
  pcl::PCLPointCloud2 pcl_cloud;
  if (pcl::io::loadPCDFile(pcd_path, pcl_cloud) == -1 || pcl_cloud.width == 0)
    throw std::runtime_error("Failed to load pcd from " + pcd_path);

  sensor_msgs::PointCloud2Ptr ros_cloud(new sensor_msgs::PointCloud2);
  pcl_conversions::moveFromPCL(pcl_cloud, *ros_cloud);
  return ros_cloud;
}

void ros_recognizer::Recognizer::initCfg()
{
  describer_cfg.reset(new DynamicConfigurator<Local3dDescriber>("~/describer"));
  matcher_cfg.reset(new DynamicConfigurator<LocalMatcher>("~/matcher"));
  verifier_cfg.reset(new DynamicConfigurator<Verifier>("~/verifier"));
}

void ros_recognizer::Recognizer::initModel()
{
  std::string model_path;
  if (node_handle_.getParam("model_path", model_path))
    model_description = desciber(*loadPCD(model_path));
  else
    throw std::runtime_error("Initial model pcd path not specified");
}

void ros_recognizer::Recognizer::initTopics()
{
  scene_subscriber_ = node_handle_.subscribe("scene", 1, &Recognizer::sceneCallback, this);
  model_cloud_service_ = node_handle_.advertiseService("set_model_from_cloud",
                                                       &Recognizer::setModelFromCloud, this);
  model_pcd_service_ = node_handle_.advertiseService("set_model_from_pcd",
                                                     &Recognizer::setModelFromPCD, this);
}

void ros_recognizer::Recognizer::refreshCfg()
{
  describer_cfg->refresh(desciber);
  matcher_cfg->refresh(matcher);
  verifier_cfg->refresh(verifier);
}

PLUGINLIB_EXPORT_CLASS(ros_recognizer::Recognizer, nodelet::Nodelet)