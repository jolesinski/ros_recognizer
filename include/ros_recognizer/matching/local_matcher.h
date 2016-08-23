
#ifndef ROS_RECOGNIZER_LOCAL_MATCHER_H
#define ROS_RECOGNIZER_LOCAL_MATCHER_H

#include <ros_recognizer/common/reconfigurable.h>
#include <ros_recognizer/MatcherConfig.h>

namespace ros_recognizer
{

class LocalMatcher : public Reconfigurable<MatcherConfig>
{

};

}


#endif //ROS_RECOGNIZER_LOCAL_MATCHER_H
