
#ifndef ROS_RECOGNIZER_LOCAL_MATCHER_H
#define ROS_RECOGNIZER_LOCAL_MATCHER_H

#include <ros_recognizer/MatcherConfig.h>
#include <ros_recognizer/common/reconfigurable.h>
#include <ros_recognizer/preprocessing/local_3d_description.h>
#include <ros_recognizer/matching/hypotheses.h>
#include <pcl/correspondence.h>

namespace ros_recognizer
{

class LocalMatcher : public Reconfigurable<MatcherConfig>
{
public:
  Hypotheses operator()(const Local3dDescription& model,
                        const Local3dDescription& scene,
                        pcl::CorrespondencesPtr& correspondences ,
                        std::vector<pcl::Correspondences>& clusters);

private:
  pcl::CorrespondencesPtr findCorrespondences(const Local3dDescription& model,
                                              const Local3dDescription& scene);

  ros_recognizer::Hypotheses groupCorrespondences(const Local3dDescription& model,
                                                  const Local3dDescription& scene,
                                                  const pcl::CorrespondencesConstPtr& correspondences,
                                                  std::vector<pcl::Correspondences>& clusters);
};

}


#endif //ROS_RECOGNIZER_LOCAL_MATCHER_H
