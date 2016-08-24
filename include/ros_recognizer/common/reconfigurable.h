
#ifndef ROS_RECOGNIZER_RECONFIGURABLE_H
#define ROS_RECOGNIZER_RECONFIGURABLE_H

#include <dynamic_reconfigure/server.h>

namespace ros_recognizer
{

template <typename ConfigType>
class Reconfigurable
{
public:
  Reconfigurable(std::string name = "");

  void reconfigure(const ConfigType& new_cfg)
  {
    cfg_ = new_cfg;
  };

  void refresh_config()
  {
    if(needs_refresh_)
      reconfigure(fresh_cfg_);
  };
protected:
  ConfigType cfg_;
  ConfigType fresh_cfg_;
  bool needs_refresh_ = false;

#ifndef ROS_RECOGNIZER_OUTSIDE_ROS_NODE
  dynamic_reconfigure::Server<ConfigType> cfg_srv_;
  void cfg_cb(ConfigType &config, uint32_t level)
  {
    fresh_cfg_ = config;
    needs_refresh_ = true;
  }
#endif
};

template <typename ConfigType>
Reconfigurable<ConfigType>::Reconfigurable(std::string name)
#ifndef ROS_RECOGNIZER_OUTSIDE_ROS_NODE
    : cfg_srv_(name)
{
  typename dynamic_reconfigure::Server<ConfigType>::CallbackType cfg_srv_cb;
  cfg_srv_cb = boost::bind(&Reconfigurable<ConfigType>::cfg_cb, this, _1, _2);
  cfg_srv_.setCallback(cfg_srv_cb);
}
#else
{
  reconfigure(cfg_.__getDefault__());
}
#endif

}

#endif //ROS_RECOGNIZER_RECONFIGURABLE_H
