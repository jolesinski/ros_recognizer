
#ifndef ROS_RECOGNIZER_LOCAL_MATCHER_H
#define ROS_RECOGNIZER_LOCAL_MATCHER_H

#include <pcl/correspondence.h>

#include <ros_recognizer/common/reconfigurable.h>
#include <ros_recognizer/description/local_3d_description.h>
#include <ros_recognizer/matching/hypotheses.h>
#include <ros_recognizer/MatcherConfig.h>

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
  pcl::CorrespondencesPtr match(const Local3dDescription& model,
                                const Local3dDescription& scene);

  ros_recognizer::Hypotheses clusterize(const Local3dDescription& model,
                                        const Local3dDescription& scene,
                                        const pcl::CorrespondencesConstPtr& correspondences,
                                        std::vector<pcl::Correspondences>& clusters);
};

}


#endif //ROS_RECOGNIZER_LOCAL_MATCHER_H
