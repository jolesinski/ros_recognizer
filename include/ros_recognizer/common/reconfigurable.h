
#ifndef ROS_RECOGNIZER_RECONFIGURABLE_H
#define ROS_RECOGNIZER_RECONFIGURABLE_H

#include <dynamic_reconfigure/server.h>

namespace ros_recognizer
{

template <typename ConfigType>
class Reconfigurable
{
public:
  Reconfigurable(std::string name);

protected:
  ConfigType cfg_;

  void reconfigure() { cfg_ = new_cfg_; };

private:
  ConfigType new_cfg_;

  dynamic_reconfigure::Server<ConfigType> cfg_srv_;
  void cfg_cb(ConfigType &config, uint32_t level) { new_cfg_ = config; }
};

template <typename ConfigType>
Reconfigurable<ConfigType>::Reconfigurable(std::string name) : cfg_srv_(name)
{
  typename dynamic_reconfigure::Server<ConfigType>::CallbackType cfg_srv_cb;
  cfg_srv_cb = boost::bind(&Reconfigurable<ConfigType>::cfg_cb, this, _1, _2);
  cfg_srv_.setCallback(cfg_srv_cb);
}

}

#endif //ROS_RECOGNIZER_RECONFIGURABLE_H
