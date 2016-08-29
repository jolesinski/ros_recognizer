
#include <ros_recognizer/verification/verifier.h>

#include <pcl/common/time.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>

#include <boost/pointer_cast.hpp>

ros_recognizer::Hypotheses
ros_recognizer::Verifier::operator()(const ros_recognizer::Hypotheses& hyps,
                                     const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene)
{
  auto refined_hyps = refine(hyps, scene);
  validate(refined_hyps, scene);

  return refined_hyps;
}

void ros_recognizer::Verifier::validate(ros_recognizer::Hypotheses& hyps,
                                        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene)
{
  pcl::ScopeTime timeit("GlobalVerification");

  std::vector<bool> hypotheses_mask(hyps.size(), false);
  pcl::GlobalHypothesesVerification<pcl::PointXYZRGBA, pcl::PointXYZRGBA> hv;

  hv.setSceneCloud(boost::const_pointer_cast<pcl::PointCloud<pcl::PointXYZRGBA>>(scene));

  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> instances;
  {
    pcl::ScopeTime timeit2("RegisteringInstances");
    for (const auto& hyp : hyps)
      instances.push_back(hyp.registered_model_);
  }
  hv.addModels(instances, true);

  hv.setInlierThreshold(cfg_.hv_inlier_th);
  hv.setOcclusionThreshold (cfg_.hv_occlusion_th);
  hv.setRegularizer(cfg_.hv_regularizer);
  hv.setRadiusClutter(cfg_.hv_rad_clutter);
  hv.setClutterRegularizer(cfg_.hv_clutter_reg);
  hv.setDetectClutter(cfg_.hv_detect_clutter);
  hv.setRadiusNormals(cfg_.hv_rad_normals);

  {
    pcl::ScopeTime timeit2("VerifyAlone");
    hv.verify();
  }
  hv.getMask(hypotheses_mask);

  for (auto idx = 0u; idx < hypotheses_mask.size(); ++idx)
    hyps.at(idx).is_valid_ = hypotheses_mask[idx];
}

ros_recognizer::Hypotheses
ros_recognizer::Verifier::refine(const ros_recognizer::Hypotheses& hyps,
                                 const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene)
{
  pcl::ScopeTime timeit("Refinement");

  Hypotheses refined_hyps;

  if(!cfg_.icp_refinement)
  {
    for(auto hyp : hyps)
    {
      hyp.registered_model_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::transformPointCloud(*hyp.input_model_, *hyp.registered_model_, hyp.pose_);
      refined_hyps.push_back(hyp);
    }
    return refined_hyps;
  }

  pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
  icp.setInputTarget(scene);
  icp.setMaxCorrespondenceDistance(cfg_.icp_corr_dist);
  icp.setMaximumIterations (cfg_.icp_max_iter);
  //icp.setTransformationEpsilon (1e-8);
  //icp.setEuclideanFitnessEpsilon (1);
  for(auto hyp : hyps)
  {
    hyp.registered_model_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    icp.setInputSource(hyp.input_model_);
    icp.align(*hyp.registered_model_, hyp.pose_);
    if (icp.hasConverged())
    {
      hyp.pose_ = icp.getFinalTransformation();
      if(cfg_.icp_two_pass)
      {
        icp.setMaxCorrespondenceDistance(cfg_.icp_corr_dist / 10);
        icp.align(*hyp.registered_model_, icp.getFinalTransformation());
        if(icp.hasConverged())
          hyp.pose_ = icp.getFinalTransformation();
        icp.setMaxCorrespondenceDistance(cfg_.icp_corr_dist);
      }
      refined_hyps.push_back(hyp);
    }
  }
  return refined_hyps;
}