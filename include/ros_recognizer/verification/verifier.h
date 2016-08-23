
#ifndef ROS_RECOGNIZER_VERIFIER_H
#define ROS_RECOGNIZER_VERIFIER_H

#include <ros_recognizer/matching/hypotheses.h>

namespace ros_recognizer
{

class Verifier
{
public:
  Hypotheses filter(const Hypotheses& hyps);
private:
  void refine();
  void verify();
};

}

#endif //ROS_RECOGNIZER_VERIFIER_H
