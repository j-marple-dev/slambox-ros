/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief sbox_msg and ROS message converter

#ifndef SLAMBOX_ROS_INCLUDE_UTILS_ROS_MSG_CONVERTER_HPP_
#define SLAMBOX_ROS_INCLUDE_UTILS_ROS_MSG_CONVERTER_HPP_

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <sbox/sbox_msgs/Odometry.hpp>
#include <sbox/sbox_msgs/PointCloud2.hpp>

namespace sbox_msgs {
nav_msgs::Odometry to_ros_msg(const Odometry &odom);
sensor_msgs::PointCloud2 to_ros_msg(const PointCloud2 &pointcloud);
}  // namespace sbox_msgs

#endif  // SLAMBOX_ROS_INCLUDE_UTILS_ROS_MSG_CONVERTER_HPP_
