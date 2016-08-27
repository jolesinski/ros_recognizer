
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
  Hypotheses operator()(const Hypotheses& hyps,
                        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene);

private:
  Hypotheses refine(const Hypotheses& hyps,
                    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene);
  void validate(Hypotheses& refined_hyps,
                const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene);
};

}

#endif //ROS_RECOGNIZER_VERIFIER_H
