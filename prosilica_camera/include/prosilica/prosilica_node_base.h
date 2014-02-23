#ifndef PROSILICA_NODE_DRIVER_H
#define PROSILICA_NODE_DRIVER_H

// ROS communication
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <polled_camera/publication_server.h>

// Diagnostics
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <self_test/self_test.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include "prosilica_camera/ProsilicaCameraConfig.h"

// Standard libs
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <sstream>

#include "prosilica/prosilica.h"
#include "prosilica/rolling_sum.h"

namespace prosilica{

/// @todo Only stream when subscribed to
class ProsilicaNodeBase
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher streaming_pub_;
  polled_camera::PublicationServer poll_srv_;
  ros::ServiceServer set_camera_info_srv_;
  ros::Subscriber trigger_sub_;
  
  // Camera
  boost::scoped_ptr<prosilica::Camera> cam_;
  prosilica::FrameStartTriggerMode trigger_mode_; /// @todo Make this property of Camera
  bool running_;
  unsigned long max_data_rate_;
  tPvUint32 sensor_width_, sensor_height_; // full resolution dimensions (maybe should be in lib)
  bool auto_adjust_stream_bytes_per_second_;

  // Hardware triggering
  std::string trig_timestamp_topic_;
  ros::Time trig_time_;

  // ROS messages
  sensor_msgs::Image img_;
  sensor_msgs::CameraInfo cam_info_;

  // Dynamic reconfigure
  typedef prosilica_camera::ProsilicaCameraConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  ReconfigureServer reconfigure_server_;
  
  // Diagnostics
  ros::Timer diagnostic_timer_;
  boost::shared_ptr<self_test::TestRunner> self_test_;
  diagnostic_updater::Updater diagnostic_;
  std::string hw_id_;
  int count_;
  double desired_freq_;
  static const int WINDOW_SIZE = 5; // remember previous 5s
  unsigned long frames_dropped_total_, frames_completed_total_;
  RollingSum<unsigned long> frames_dropped_acc_, frames_completed_acc_;
  unsigned long packets_missed_total_, packets_received_total_;
  RollingSum<unsigned long> packets_missed_acc_, packets_received_acc_;

  // So we don't get burned by auto-exposure
  unsigned long last_exposure_value_;
  int consecutive_stable_exposures_;

public:
  ProsilicaNodeBase(ros::NodeHandle& node_handle, ros::NodeHandle& local_nh, const std::string& node_name);

  void configure(Config& config, uint32_t level);

  ~ProsilicaNodeBase();

  void syncInCallback (const std_msgs::HeaderConstPtr& msg);
  
  void start();

  void stop();

  void pollCallback(polled_camera::GetPolledImage::Request& req,
                    polled_camera::GetPolledImage::Response& rsp,
                    sensor_msgs::Image& image, sensor_msgs::CameraInfo& info);

  static bool frameToImage(tPvFrame* frame, sensor_msgs::Image &image);
  
  bool processFrame(tPvFrame* frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info);
  
  void publishImage(tPvFrame* frame);

  void loadIntrinsics();

  bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req,
                     sensor_msgs::SetCameraInfo::Response& rsp);

  void normalizeCallback(tPvFrame* frame);
  
  void normalizeExposure();

  /////////////////
  // Diagnostics //
  /////////////////
  
  void runDiagnostics();
  
  void freqStatus(diagnostic_updater::DiagnosticStatusWrapper& status);

  void frameStatistics(diagnostic_updater::DiagnosticStatusWrapper& status);

  void packetStatistics(diagnostic_updater::DiagnosticStatusWrapper& status);

  void packetErrorStatus(diagnostic_updater::DiagnosticStatusWrapper& status);


  ////////////////
  // Self tests //
  ////////////////

  // Try to load camera name, etc. Should catch gross communication failure.
  void infoTest(diagnostic_updater::DiagnosticStatusWrapper& status);

  // Test validity of all attribute values.
  void attributeTest(diagnostic_updater::DiagnosticStatusWrapper& status);

  // Try to capture an image.
  void imageTest(diagnostic_updater::DiagnosticStatusWrapper& status);
};

}

#endif
