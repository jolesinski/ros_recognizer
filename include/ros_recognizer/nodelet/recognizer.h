
#ifndef ROS_RECOGNIZER_RECOGNIZER_H
#define ROS_RECOGNIZER_RECOGNIZER_H

#include <nodelet/nodelet.h>

namespace ros_recognizer
{

class Recognizer : public nodelet::Nodelet
{
public:
  virtual void onInit();
};

}


#endif //ROS_RECOGNIZER_RECOGNIZER_H
