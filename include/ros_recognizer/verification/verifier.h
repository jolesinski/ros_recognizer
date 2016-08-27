
#ifndef ROS_RECOGNIZER_VERIFIER_H
#define ROS_RECOGNIZER_VERIFIER_H

#include <ros_recognizer/VerifierConfig.h>
#include <ros_recognizer/common/reconfigurable.h>
#include <ros_recognizer/matching/hypotheses.h>

namespace ros_recognizer
{

class Verifier : public Reconfigurable<VerifierConfig>
{
public:
  Hypotheses verify(const ros_recognizer::Hypotheses& refined_hyps,
                    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene);
private:
  Hypotheses refine(const ros_recognizer::Hypotheses& hyps,
                    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene);
};

}

#endif //ROS_RECOGNIZER_VERIFIER_H
