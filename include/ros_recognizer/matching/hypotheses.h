
#ifndef ROS_RECOGNIZER_HYPOTHESES_H
#define ROS_RECOGNIZER_HYPOTHESES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ros_recognizer
{

using Pose = Eigen::Matrix4f;
using Poses = std::vector<Pose, Eigen::aligned_allocator<Pose>>;

struct Hypothesis
{
  Eigen::Matrix4f pose_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr registered_model_;
  bool is_valid_ = false;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using Hypotheses = std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>;

}

#endif //ROS_RECOGNIZER_HYPOTHESES_H
