
#ifndef ROS_RECOGNIZER_RECONFIGURABLE_H
#define ROS_RECOGNIZER_RECONFIGURABLE_H

#include <dynamic_reconfigure/server.h>

namespace ros_recognizer
{

template <typename ConfigType>
class Reconfigurable
{
protected:
  ConfigType cfg_;

public:
  Reconfigurable() { reconfigure(cfg_.__getDefault__()); };

  void reconfigure(const ConfigType& new_cfg) { cfg_ = new_cfg; };
};

template <typename ConfigType>
class DynamicallyReconfigurable : public Reconfigurable<ConfigType>
{
public:
  DynamicallyReconfigurable(std::string name);

private:
  dynamic_reconfigure::Server<ConfigType> cfg_srv_;
  void cfg_cb(ConfigType &config, uint32_t level) { reconfigure(config); }
};

template <typename ConfigType>
DynamicallyReconfigurable<ConfigType>::DynamicallyReconfigurable(std::string name) : cfg_srv_(name)
{
  typename dynamic_reconfigure::Server<ConfigType>::CallbackType cfg_srv_cb;
  cfg_srv_cb = boost::bind(&DynamicallyReconfigurable<ConfigType>::cfg_cb, this, _1, _2);
  cfg_srv_.setCallback(cfg_srv_cb);
}

}

#endif //ROS_RECOGNIZER_RECONFIGURABLE_H
