
#include <ros_recognizer/matching/local_matcher.h>

#include <pcl/common/time.h>
#define PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>
#undef PCL_NO_PRECOMPILE
#include <pcl/recognition/cg/hough_3d.h>

ros_recognizer::Hypotheses
ros_recognizer::LocalMatcher::operator()(const ros_recognizer::Local3dDescription& model,
                                                               const ros_recognizer::Local3dDescription& scene)
{
  auto corrs = findCorrespondences(model, scene);
  std::cout << "Correspondences found: " << corrs->size() << std::endl;
  return groupCorrespondences(model, scene, corrs);
}

pcl::CorrespondencesPtr
ros_recognizer::LocalMatcher::findCorrespondences(const ros_recognizer::Local3dDescription& model,
                                                  const ros_recognizer::Local3dDescription& scene)
{
  pcl::ScopeTime timeit("Correspondences");
  std::vector<pcl::Correspondences> per_point_corrs(scene.descriptors_->size());
  pcl::KdTreeFLANN<pcl::SHOT1344, ::flann::L1<float>> match_search;
  match_search.setInputCloud(model.descriptors_);
  std::vector<int> neigh_indices(10);
  std::vector<float> neigh_sqr_dists(10);

  // For each scene keypoint descriptor, find nearest neighbor into the
  // model keypoints descriptor cloud and add it to the correspondences vector.
#pragma omp parallel for shared (per_point_corrs) private (neigh_indices, neigh_sqr_dists) num_threads(8)
  for (size_t descr_idx = 0; descr_idx < scene.descriptors_->size(); ++descr_idx)
  {
    if (!pcl_isfinite ((*scene.descriptors_)[descr_idx].descriptor[0])) //skipping NaNs
      continue;

    int found_neighs = match_search.radiusSearch(
        scene.descriptors_->at(descr_idx),
        cfg_.corr_distance,
        neigh_indices,
        neigh_sqr_dists,
        10
    );

    for(int corr_idx = 0; corr_idx < found_neighs; corr_idx++)
    {
      pcl::Correspondence corr(neigh_indices[corr_idx],
                               static_cast<int>(descr_idx),
                               neigh_sqr_dists[corr_idx]);

      per_point_corrs[descr_idx].push_back(corr);
    }
  }

  //Fill output vector
  pcl::CorrespondencesPtr output_corrs(new pcl::Correspondences);
  size_t total_size{ 0 };
  for (auto const& items: per_point_corrs){
    total_size += items.size();
  }

  output_corrs->reserve(total_size);
  for (auto& items: per_point_corrs){
    std::move(items.begin(), items.end(), std::back_inserter(*output_corrs));
  }
  return output_corrs;
}

ros_recognizer::Hypotheses
ros_recognizer::LocalMatcher::groupCorrespondences(const ros_recognizer::Local3dDescription& model,
                                                   const ros_recognizer::Local3dDescription& scene,
                                                   const pcl::CorrespondencesConstPtr& correspondences)
{
  pcl::ScopeTime timeit("Clustering");

  //TODO: rewrite to use color (from v4r)
  pcl::Hough3DGrouping<pcl::PointXYZRGBA, pcl::PointXYZRGBA> clusterer;
  clusterer.setHoughBinSize (cfg_.cluster_size);
  clusterer.setHoughThreshold (cfg_.cluster_thresh);
  clusterer.setUseInterpolation (true);
  clusterer.setUseDistanceWeight (false);

  clusterer.setInputCloud (model.keypoints_);
  clusterer.setInputRf (model.ref_frames_);
  clusterer.setSceneCloud (scene.keypoints_);
  clusterer.setSceneRf (scene.ref_frames_);
  clusterer.setModelSceneCorrespondences (correspondences);

  Poses poses;
  clusterer.recognize(poses);

  ros_recognizer::Hypotheses hypotheses;
  for (const Pose& pose : poses)
  {
    Hypothesis hyp;
    hyp.pose_ = pose;
    hyp.input_model_ = model.input_;
    hypotheses.push_back(hyp);
  }
  return hypotheses;
}
