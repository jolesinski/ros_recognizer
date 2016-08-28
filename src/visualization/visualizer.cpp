
#include <ros_recognizer/visualization/visualizer.h>
#include <pcl/common/transforms.h>

ros_recognizer::Visualizer::Visualizer() : vis_("ros_recognizer_visualizer")
{
  vis_.setBackgroundColor (0.1, 0.1, 0.1);
  vis_.addCoordinateSystem (0.2);
  vis_.initCameraParameters ();
  vis_.resetCamera();
}

void ros_recognizer::Visualizer::setModel(const ros_recognizer::Local3dDescription& model)
{
  model_ = shiftModel(model);
}

void ros_recognizer::Visualizer::setScene(const ros_recognizer::Local3dDescription& scene)
{
  scene_ = scene;
}

void ros_recognizer::Visualizer::render()
{
  if (cfg_.show_model)
    showDescription(model_, "model");
  if (cfg_.show_scene)
    showDescription(scene_, "scene");

  vis_.spinOnce(SPIN_MS);
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
  vis_.removePointCloud(id);

  if(cloud != nullptr)
  {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> colorInput(cloud);
    vis_.addPointCloud(cloud, colorInput, id);
  }
}

void ros_recognizer::Visualizer::showNormals(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input,
                                             const pcl::PointCloud<pcl::Normal>::Ptr& normals,
                                             const std::string& id)
{
  vis_.removePointCloud("normals");

  if(input != nullptr && normals != nullptr && cfg_.show_normals)
  {
    vis_.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(input, normals, 10, 0.02, "normals");
  }
}

void ros_recognizer::Visualizer::showKeypoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                                               const std::string& id)
{
  vis_.removePointCloud(id);

  if(cloud != nullptr && cfg_.show_keypoints)
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> colorKeypoints(cloud, 255, 0, 0);
    vis_.addPointCloud(cloud, colorKeypoints, id);
    vis_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);
  }
}

ros_recognizer::Local3dDescription
ros_recognizer::Visualizer::shiftModel(const ros_recognizer::Local3dDescription& model)
{
  ros_recognizer::Local3dDescription off_scene_descr;

  pcl::transformPointCloud (*model.input_,
                            *off_scene_descr.input_,
                            Eigen::Vector3f(-.5,0,0),
                            Eigen::Quaternionf(0.5, 0, 0.86603, 0));
  pcl::transformPointCloud (*model.keypoints_,
                            *off_scene_descr.keypoints_,
                            Eigen::Vector3f (-.5,0,0),
                            Eigen::Quaternionf (0.5, 0, 0.86603, 0));

  off_scene_descr.normals_ = model.normals_;
  off_scene_descr.descriptors_ = model.descriptors_;
  off_scene_descr.ref_frames_ = model.ref_frames_;

  return off_scene_descr;
}