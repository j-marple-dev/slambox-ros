/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief SLAMBOX ROS Driver client node

#include <glog/logging.h>
#include <ros/ros.h>

#include "applications/driver_client.hpp"

/// @brief Main node for SLAMBOX Driver Server
/// @param argc Number of arguments
/// @param argv arguments
///
/// @return 0
int main(int argc, char **argv) {
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "SLAMBOX_ROS_driver_node");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  sbox::SLAMBOXDriverClient driver_client(nh, pnh);

  LOG(INFO) << "SLAMBOX client node is running.";
  ros::spin();

  return 0;
}
