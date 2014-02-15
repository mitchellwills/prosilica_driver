#include "prosilica/prosilica_node_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "prosilica_driver");

  ros::NodeHandle nh("camera");
  ros::NodeHandle pnh("~");
  prosilica::ProsilicaNodeDriver pn(nh, pnh);
  ros::spin();

  return 0;
}
