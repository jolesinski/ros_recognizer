#include <ros_recognizer/refining/refiner.h>

#include <pcl/common/time.h>
#include <pcl/registration/icp.h>

ros_recognizer::Hypotheses
ros_recognizer::HypothesisRefiner::refine(const ros_recognizer::Hypotheses& hyps,
                                          const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene)
{
  pcl::ScopeTime timeit("Refinement");

  Hypotheses refined_hyps;
  pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
  icp.setInputSource(model);
  icp.setInputTarget(scene);
  icp.setMaxCorrespondenceDistance(cfg_.icp_corr_dist);
  icp.setMaximumIterations (cfg_.icp_max_iter);
  //icp.setTransformationEpsilon (1e-8);
  //icp.setEuclideanFitnessEpsilon (1);
  for(const auto& hyp : hyps)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_registered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    icp.align(*model_registered, hyp.pose_);
    if (icp.hasConverged())
    {
      Hypothesis refined_hyp;
      refined_hyp.pose_ = icp.getFinalTransformation();
      if(cfg_.icp_two_pass)
      {
        icp.setMaxCorrespondenceDistance(cfg_.icp_corr_dist / 10);
        icp.align(*model_registered, icp.getFinalTransformation());
        if(icp.hasConverged())
          refined_hyp.pose_ = icp.getFinalTransformation();
        icp.setMaxCorrespondenceDistance(cfg_.icp_corr_dist);
      }
      refined_hyps.push_back(refined_hyp);
    }
  }
  return refined_hyps;
}