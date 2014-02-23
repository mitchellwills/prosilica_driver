#include "prosilica/prosilica_node_base.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "prosilica_driver");

  ros::NodeHandle nh("camera");
  ros::NodeHandle pnh("~");
  prosilica::ProsilicaNodeBase pn(nh, pnh, ros::this_node::getName());
  ros::spin();

  return 0;
}
