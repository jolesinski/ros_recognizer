
#ifndef ROS_RECOGNIZER_CORRESPONDENCE_ROS_H_H
#define ROS_RECOGNIZER_CORRESPONDENCE_ROS_H_H

#include <pcl/correspondence.h>
#include <ros_recognizer/Correspondence.h>
#include <ros_recognizer/CorrespondenceArray.h>
#include <ros_recognizer/CorrespondenceClusters.h>

namespace ros_recognizer
{

inline pcl::Correspondence toPCL(const Correspondence& msg)
{
  return pcl::Correspondence(msg.index_query, msg.index_match, msg.distance);
}

inline Correspondence fromPCL(const pcl::Correspondence& corr)
{
  Correspondence corr_msg;
  corr_msg.index_query = corr.index_query;
  corr_msg.index_match = corr.index_match;
  corr_msg.distance = corr.distance;
  return corr_msg;
}

inline pcl::Correspondences toPCL(const CorrespondenceArray& msg)
{
  pcl::Correspondences corrs;
  for(const auto& corr_msg : msg.correspondences)
    corrs.emplace_back(corr_msg.index_query, corr_msg.index_match, corr_msg.distance);

  return corrs;
}

inline CorrespondenceArray fromPCL(const pcl::Correspondences& corrs)
{
  CorrespondenceArray corrs_msg;
  for(const auto& corr : corrs)
    corrs_msg.correspondences.emplace_back(fromPCL(corr));

  return corrs_msg;
}

inline std::vector<pcl::Correspondences> toPCL(const CorrespondenceClusters& msg)
{
  std::vector<pcl::Correspondences> clusters;
  for(const auto& cluster_msg : msg.clusters)
    clusters.emplace_back(toPCL(cluster_msg));

  return clusters;
}

inline CorrespondenceClusters fromPCL(const std::vector<pcl::Correspondences>& clusters)
{
  CorrespondenceClusters msg;
  for(const auto& cluster : clusters)
    msg.clusters.emplace_back(fromPCL(cluster));

  return msg;
}

}

#endif //ROS_RECOGNIZER_CORRESPONDENCE_ROS_H_H
