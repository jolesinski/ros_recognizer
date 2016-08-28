
#ifndef ROS_RECOGNIZER_VISUALIZER_H
#define ROS_RECOGNIZER_VISUALIZER_H

#include <ros_recognizer/common/reconfigurable.h>
#include <ros_recognizer/VisualizerConfig.h>

namespace ros_recognizer
{

class Visualizer : public Reconfigurable<VisualizerConfig>
{

};

}

#endif //ROS_RECOGNIZER_VISUALIZER_H
