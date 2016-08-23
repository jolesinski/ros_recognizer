
#ifndef ROS_RECOGNIZER_HYPOTHESES_H
#define ROS_RECOGNIZER_HYPOTHESES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ros_recognizer
{

struct Hypotheses
{
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses_;
};

}

#endif //ROS_RECOGNIZER_HYPOTHESES_H
