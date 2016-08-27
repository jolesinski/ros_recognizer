
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

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using Hypotheses = std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>;

}

#endif //ROS_RECOGNIZER_HYPOTHESES_H
