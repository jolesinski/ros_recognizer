
#include <ros_recognizer/matching/local_matcher.h>

#include <pcl/common/time.h>
#define PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>
#undef PCL_NO_PRECOMPILE
#include <pcl/recognition/cg/hough_3d.h>

ros_recognizer::Hypotheses
ros_recognizer::LocalMatcher::match(const ros_recognizer::Local3dDescription& model,
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
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
  pcl::KdTreeFLANN<pcl::SHOT1344, ::flann::L1<float>> match_search;
  match_search.setInputCloud(model.descriptors_);
  std::vector<int> neigh_indices(10);
  std::vector<float> neigh_sqr_dists(10);

  // For each scene keypoint descriptor, find nearest neighbor into the
  // model keypoints descriptor cloud and add it to the correspondences vector.
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
      correspondences->push_back(corr);
    }
  }
  return correspondences;
}

ros_recognizer::Hypotheses
ros_recognizer::LocalMatcher::groupCorrespondences(const ros_recognizer::Local3dDescription& model,
                                                   const ros_recognizer::Local3dDescription& scene,
                                                   const pcl::CorrespondencesConstPtr& correspondences)
{
  pcl::ScopeTime timeit("Clustering");
  ros_recognizer::Hypotheses hypotheses;

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

  clusterer.recognize(hypotheses.poses_);
  return hypotheses;
}
