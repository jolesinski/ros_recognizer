
#ifndef ROS_RECOGNIZER_HYPOTHESES_H
#define ROS_RECOGNIZER_HYPOTHESES_H

namespace ros_recognizer
{

struct Hypotheses
{
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses_;
};

}

#endif //ROS_RECOGNIZER_HYPOTHESES_H
