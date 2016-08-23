#include <ros_recognizer/preprocessing/local_3d_describer.h>
#include <pcl/common/time.h>
#include <pcl/features/board.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl_conversions/pcl_conversions.h>

ros_recognizer::Local3dDescription
ros_recognizer::Local3dDescriber::describe(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ros_recognizer::Local3dDescription description;
  {
    pcl::ScopeTime timeit("CloudLoad");
    pcl::fromROSMsg(*cloud_msg, *description.input_);
  }
  std::cout << "Data count: " << description.input_->size() << std::endl;

  auto normals_loaded = std::any_of(std::begin(cloud_msg->fields), std::end(cloud_msg->fields),
                                    [](const sensor_msgs::PointField& field)
                                    { return field.name == "normal_x"; });
  if(normals_loaded)
    pcl::fromROSMsg(*cloud_msg, *description.normals_);
  else
    computeNormals(description);

  //computeResolution(description);
  computeKeypoints(description);
  //computeRefFrames(description);
  computeDescriptors(description);
  return description;
}

void ros_recognizer::Local3dDescriber::computeNormals(ros_recognizer::Local3dDescription& data)
{
  pcl::ScopeTime timeit("Normals");

  if(data.input_->isOrganized())
  {
    PCL_INFO("IS ORGANIZED\n");
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> norm_est;
    norm_est.setNormalEstimationMethod(norm_est.AVERAGE_3D_GRADIENT);
    norm_est.setMaxDepthChangeFactor(0.02);
    norm_est.setDepthDependentSmoothing(true);
    norm_est.setNormalSmoothingSize(10);
    norm_est.setBorderPolicy(norm_est.BORDER_POLICY_MIRROR);
    norm_est.setInputCloud(data.input_);
    norm_est.compute(*data.normals_);
  }
  else
  {
    PCL_INFO("IS NOT ORGANIZED\n");
    pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> norm_est(cfg_.omp_threads);
    norm_est.setRadiusSearch(cfg_.descr_rad);
    norm_est.setInputCloud(data.input_);
    norm_est.compute(*data.normals_);
  }
}

void ros_recognizer::Local3dDescriber::computeResolution(ros_recognizer::Local3dDescription& data)
{
  pcl::ScopeTime timeit("Resolution");
  int n_points = 0;
  std::vector<int> indices(2);
  std::vector<float> sqr_distances(2);

  pcl::search::OrganizedNeighbor<pcl::PointXYZRGBA> organized_tree;
  pcl::search::KdTree<pcl::PointXYZRGBA> kdtree;
  pcl::search::Search<pcl::PointXYZRGBA>* tree;


  if(data.input_->isOrganized())
    tree = &organized_tree;
  else
    tree = &kdtree;

  tree->setInputCloud(data.input_);
  for(const auto& point : data.input_->points)
  {
    if (!pcl_isfinite(point.x))
      continue;

    //Considering the second neighbor since the first is the point itself.
    if (tree->nearestKSearch(point, 2, indices, sqr_distances) == 2)
    {
      data.input_resolution_ += sqrt(sqr_distances[1]);
      ++n_points;
    }
  }

  if (n_points != 0)
    data.input_resolution_ /= n_points;

}

void ros_recognizer::Local3dDescriber::computeKeypoints(ros_recognizer::Local3dDescription& data)
{
  pcl::ScopeTime timeit("Keypoints");
  pcl::UniformSampling<pcl::PointXYZRGBA> uniform_sampling;
  uniform_sampling.setInputCloud(data.input_);
  uniform_sampling.setRadiusSearch(cfg_.uniform_radius);
  uniform_sampling.filter(*data.keypoints_);
}

void ros_recognizer::Local3dDescriber::computeRefFrames(ros_recognizer::Local3dDescription& data)
{
  pcl::ScopeTime timeit("ReferenceFrames");
  pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::ReferenceFrame> rf_est;
  rf_est.setFindHoles(true);
  rf_est.setRadiusSearch(cfg_.descr_rad);
  rf_est.setInputCloud(data.keypoints_);
  rf_est.setInputNormals(data.normals_);
  rf_est.setSearchSurface(data.input_);
  rf_est.compute(*data.ref_frames_);
}

void ros_recognizer::Local3dDescriber::computeDescriptors(ros_recognizer::Local3dDescription& data)
{
  pcl::ScopeTime timeit("Descriptors");
  pcl::SHOTColorEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> descr_est;
  descr_est.setRadiusSearch(cfg_.descr_rad);
  descr_est.setNumberOfThreads(cfg_.omp_threads);
  descr_est.setInputCloud(data.keypoints_);
  descr_est.setInputNormals(data.normals_);
  //descr_est.setInputReferenceFrames(data.ref_frames_);
  descr_est.setLRFRadius(cfg_.descr_rad);
  descr_est.setSearchSurface(data.input_);
  descr_est.compute(*data.descriptors_);
  *data.ref_frames_ = *descr_est.getInputReferenceFrames();
}
