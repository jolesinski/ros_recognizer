
#include <ros_recognizer/verification/verifier.h>

#include <pcl/common/time.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>

#include <boost/pointer_cast.hpp>

ros_recognizer::Hypotheses
ros_recognizer::Verifier::verify(const ros_recognizer::Hypotheses& hyps,
                                 const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& model,
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
    {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr instance(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::transformPointCloud(*model, *instance, hyp.pose_);
      instances.push_back(instance);
    }
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

  Hypotheses verified_hyps;
  for (auto idx = 0u; idx < hypotheses_mask.size(); ++idx)
  {
    if (hypotheses_mask[idx])
    {
      std::cout << "Instance " << idx << " is GOOD! <---" << std::endl;
      verified_hyps.push_back(hyps[idx]);
    }
    else
      std::cout << "Instance " << idx << " is bad!" << std::endl;
  }

  return verified_hyps;
}

ros_recognizer::Hypotheses
ros_recognizer::Verifier::refine(const ros_recognizer::Hypotheses& hyps,
                                 const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& model,
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