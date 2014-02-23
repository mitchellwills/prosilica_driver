#include <nodelet/nodelet.h>
#include "prosilica/prosilica_node_base.h"
#include <pluginlib/class_list_macros.h>

namespace prosilica{

class ProsilicaNodelet: public nodelet::Nodelet
{
public:
  ProsilicaNodelet()
  {}

  ~ProsilicaNodelet()
  {
  }

private:
  virtual void onInit();

  boost::shared_ptr<prosilica::ProsilicaNodeBase> dvr_;
};

/** Nodelet initialization.
 *
 *  @note MUST return immediately.
 */
void ProsilicaNodelet::onInit()
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());
  ros::NodeHandle node(getNodeHandle(), "camera");
  dvr_.reset(new prosilica::ProsilicaNodeBase(node, priv_nh, getName()));
}

}

// Register this plugin with pluginlib.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(prosilica, ProsilicaNodelet,
                        prosilica::ProsilicaNodelet, nodelet::Nodelet);
