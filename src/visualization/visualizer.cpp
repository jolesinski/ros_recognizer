
#include <ros_recognizer/visualization/visualizer.h>
#include <pcl/common/transforms.h>

void ros_recognizer::Visualizer::initVis()
{
  vis_.reset(new pcl::visualization::PCLVisualizer);

  vis_->setBackgroundColor (0.1, 0.1, 0.1);
  vis_->addCoordinateSystem (0.2);
  vis_->initCameraParameters ();
  vis_->resetCamera();
}

void ros_recognizer::Visualizer::setModel(const ros_recognizer::Local3dDescription& model)
{
  model_ = shiftModel(model);
  clearResults();
  needs_redrawal_ = true;
}

void ros_recognizer::Visualizer::setScene(const ros_recognizer::Local3dDescription& scene)
{
  scene_ = scene;
  clearResults();
  needs_redrawal_ = true;
}

void ros_recognizer::Visualizer::setCorrespondences(const pcl::Correspondences& corrs)
{
  corrs_ = corrs;
  needs_redrawal_ = true;
}

void ros_recognizer::Visualizer::setClusters(const std::vector<pcl::Correspondences>& clusters)
{
  clusters_ = clusters;
  needs_redrawal_ = true;
}

void ros_recognizer::Visualizer::setHypotheses(const ros_recognizer::Hypotheses& hyps, bool valid)
{
  if (valid)
    true_hypotheses_ = hyps;
  else
    false_hypotheses_ = hyps;
  needs_redrawal_ = true;
}


void ros_recognizer::Visualizer::render(bool force_redrawal)
{
  if(vis_ == nullptr)
    initVis();

  if(needs_redrawal_ || force_redrawal)
  {
    vis_->removeAllPointClouds();

    if (cfg_.show_model)
      showDescription(model_, "model");
    if (cfg_.show_scene)
      showDescription(scene_, "scene");
    if (cfg_.show_corrs)
      showCorrespondences();
    if (cfg_.show_true_hypotheses)
      showHypotheses(true_hypotheses_, true);
    if (cfg_.show_false_hypotheses)
      showHypotheses(false_hypotheses_, false);
  }

  needs_redrawal_ = false;
  vis_->spinOnce(SPIN_MS);
}

void ros_recognizer::Visualizer::showDescription(const ros_recognizer::Local3dDescription& descr,
                                                 const std::string& id_prefix)
{
  showInput(descr.input_, id_prefix + "_input");
  showNormals(descr.input_, descr.normals_, id_prefix + "_normals");
  showKeypoints(descr.keypoints_, id_prefix + "_keypoints");
}

void ros_recognizer::Visualizer::showInput(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                                           const std::string& id)
{
  if(!cloud->empty())
  {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> colorInput(cloud);
    vis_->addPointCloud(cloud, colorInput, id);
  }
}

void ros_recognizer::Visualizer::showNormals(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input,
                                             const pcl::PointCloud<pcl::Normal>::Ptr& normals,
                                             const std::string& id)
{
  if(!input->empty() && !normals->empty() && cfg_.show_normals)
    vis_->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(input, normals, 10, 0.02, id);
}

void ros_recognizer::Visualizer::showKeypoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                                               const std::string& id)
{
  if(!cloud->empty() && cfg_.show_keypoints)
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> colorKeypoints(cloud, 126, 0, 255);
    vis_->addPointCloud(cloud, colorKeypoints, id);
    vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);
  }
}

ros_recognizer::Local3dDescription
ros_recognizer::Visualizer::shiftModel(const ros_recognizer::Local3dDescription& model)
{
  ros_recognizer::Local3dDescription off_scene_descr;

  pcl::PointCloud<pcl::PointNormal> off_scene_pointNormals;
  pcl::copyPointCloud(*model.input_, off_scene_pointNormals);
  pcl::copyPointCloud(*model.normals_, off_scene_pointNormals);

  pcl::transformPointCloud (*model.input_,
                            *off_scene_descr.input_,
                            Eigen::Vector3f(-.5,0,0),
                            Eigen::Quaternionf(0.5, 0, 0.86603, 0));
  pcl::transformPointCloud (*model.keypoints_,
                            *off_scene_descr.keypoints_,
                            Eigen::Vector3f (-.5,0,0),
                            Eigen::Quaternionf (0.5, 0, 0.86603, 0));
  pcl::transformPointCloudWithNormals(off_scene_pointNormals,
                                      off_scene_pointNormals,
                                      Eigen::Vector3f (-.5,0,0),
                                      Eigen::Quaternionf (0.5, 0, 0.86603, 0));

  pcl::copyPointCloud(off_scene_pointNormals, *off_scene_descr.normals_);

  off_scene_descr.descriptors_ = model.descriptors_;
  off_scene_descr.ref_frames_ = model.ref_frames_;

  return off_scene_descr;
}

void ros_recognizer::Visualizer::showHypotheses(const ros_recognizer::Hypotheses& hypotheses, bool valid)
{
  auto cnt = 0u;
  for(const auto& hyp : hypotheses)
  {
    using colorHandler = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>;
    auto color = valid ? colorHandler(hyp.registered_model_, 0, 255, 0)
                       : colorHandler(hyp.registered_model_, 255, 0, 0);
    std::string id = valid ? "true_hyps_" : "false_hyps_";
    vis_->addPointCloud(hyp.registered_model_, color, id + std::to_string(cnt++));
  }
}

void ros_recognizer::Visualizer::showCorrespondences()
{
  static const std::string prefix = "corresp_";
  static auto cnt = 0u;

  while (cnt)
    vis_->removeCorrespondences(prefix + std::to_string(--cnt));

  if (cfg_.show_clusters)
  {
    for(const auto& cluster : clusters_)
    {
      vis_->addCorrespondences<pcl::PointXYZRGBA>(model_.keypoints_,
                                                  scene_.keypoints_,
                                                  cluster,
                                                  prefix + std::to_string(cnt++));
    }
  }
  else
  {
    vis_->addCorrespondences<pcl::PointXYZRGBA>(model_.keypoints_,
                                                scene_.keypoints_,
                                                corrs_,
                                                prefix + std::to_string(cnt++));
  }
}

void ros_recognizer::Visualizer::clearResults()
{
  true_hypotheses_.clear();
  false_hypotheses_.clear();
  corrs_.clear();
  clusters_.clear();
}