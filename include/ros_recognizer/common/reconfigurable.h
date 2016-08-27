
#ifndef ROS_RECOGNIZER_RECONFIGURABLE_H
#define ROS_RECOGNIZER_RECONFIGURABLE_H

namespace ros_recognizer
{

template <typename ConfigType>
class Reconfigurable
{
public:
  using Cfg = ConfigType;

  Reconfigurable()
  {
    reconfigure(cfg_.__getDefault__());
  }

  void reconfigure(const Cfg& new_cfg)
  {
    cfg_ = new_cfg;
  };

protected:
  Cfg cfg_;
};

}

#endif //ROS_RECOGNIZER_RECONFIGURABLE_H
