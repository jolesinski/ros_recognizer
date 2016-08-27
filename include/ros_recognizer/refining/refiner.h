
#ifndef ROS_RECOGNIZER_REFINER_H
#define ROS_RECOGNIZER_REFINER_H

#include <ros_recognizer/RefinerConfig.h>
#include <ros_recognizer/common/reconfigurable.h>
#include <ros_recognizer/matching/hypotheses.h>

namespace ros_recognizer
{


class HypothesisRefiner : public Reconfigurable<RefinerConfig>
{
public:
Hypotheses refine(const Hypotheses& hyps,
                  const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& scene);
};

}


#endif //ROS_RECOGNIZER_REFINER_H
