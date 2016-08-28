
#ifndef ROS_RECOGNIZER_DYNAMIC_CONFIGURATOR_H
#define ROS_RECOGNIZER_DYNAMIC_CONFIGURATOR_H

#include <dynamic_reconfigure/server.h>

namespace ros_recognizer
{

template <typename ReconfigurableType>
class DynamicConfigurator
{
public:
  DynamicConfigurator(const std::string& ns);

  bool refresh(ReconfigurableType& cfg_receiver)
  {
    if(needs_refresh_)
    {
      cfg_receiver.reconfigure(fresh_cfg_);
      needs_refresh_ = false;
      return true;
    }

    return false;
  };
protected:
  using Cfg = typename ReconfigurableType::Cfg;

  Cfg fresh_cfg_;
  bool needs_refresh_ = false;

  dynamic_reconfigure::Server<Cfg> cfg_srv_;
  void cfg_cb(Cfg &config, uint32_t level)
  {
    fresh_cfg_ = config;
    needs_refresh_ = true;
  }
};

template <typename ReconfigurableType>
DynamicConfigurator<ReconfigurableType>::DynamicConfigurator(const std::string& ns)
    : cfg_srv_(ns)
{
  typename dynamic_reconfigure::Server<Cfg>::CallbackType cfg_srv_cb;
  cfg_srv_cb = boost::bind(&DynamicConfigurator<ReconfigurableType>::cfg_cb, this, _1, _2);
  cfg_srv_.setCallback(cfg_srv_cb);
}


}

#endif //ROS_RECOGNIZER_DYNAMIC_CONFIGURATOR_H
