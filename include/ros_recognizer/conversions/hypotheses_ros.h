
#ifndef ROS_RECOGNIZER_HYPOTHESES_TO_ROS_H
#define ROS_RECOGNIZER_HYPOTHESES_TO_ROS_H

#include <geometry_msgs/PoseArray.h>
#include <ros_recognizer/matching/hypotheses.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/transforms.h>

namespace ros_recognizer
{

inline Hypotheses convertMsgToHypotheses(const geometry_msgs::PoseArray msg,
                                         pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr model,
                                         bool is_valid)
{
  Hypotheses hyps;
  for(const auto& pose : msg.poses)
  {
    Hypothesis hypothesis;
    Eigen::Affine3d affine;
    tf::poseMsgToEigen(pose, affine);
    hypothesis.pose_ = affine.matrix().cast<float>();
    hypothesis.input_model_ = model;
    hypothesis.registered_model_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*model, *hypothesis.registered_model_, hypothesis.pose_);
    hypothesis.is_valid_ = is_valid;
    hyps.push_back(hypothesis);
  }

  return hyps;
}

inline void convertHypothesesToMsg(const ros_recognizer::Hypotheses& hyps,
                                   geometry_msgs::PoseArray& valid_poses,
                                   geometry_msgs::PoseArray& false_poses)
{
  for (const auto& hyp : hyps)
  {
    geometry_msgs::Pose pose;
    Eigen::Affine3d affine(hyp.pose_.cast<double>());
    tf::poseEigenToMsg(affine, pose);
    if(hyp.is_valid_)
      valid_poses.poses.push_back(pose);
    else
      false_poses.poses.push_back(pose);
  }
}

}

#endif //ROS_RECOGNIZER_HYPOTHESES_TO_ROS_H
