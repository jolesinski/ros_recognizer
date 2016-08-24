
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
  Hypotheses filter(const Hypotheses& hyps,
                    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& model,
                    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene);

private:
  Hypotheses refine(const Hypotheses& hyps,
                    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& model,
                    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene,
                    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr>& instances);

  Hypotheses verify(const ros_recognizer::Hypotheses& hyps,
                    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene,
                    const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr>& instances);
};

}

#endif //ROS_RECOGNIZER_VERIFIER_H
