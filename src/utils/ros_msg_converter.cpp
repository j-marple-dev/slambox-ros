/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief sbox_msg and ROS message converter

#include "utils/ros_msg_converter.hpp"

namespace sbox_msgs {
nav_msgs::Odometry to_ros_msg(const Odometry &odom) {
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time(odom.timestamp_sec, odom.timestamp_nsec);

  odom_msg.header.frame_id = "camera_init";
  odom_msg.child_frame_id = "body";

  odom_msg.pose.pose.position.x = odom.pose.position.x;
  odom_msg.pose.pose.position.y = odom.pose.position.y;
  odom_msg.pose.pose.position.z = odom.pose.position.z;

  odom_msg.pose.pose.orientation.x = odom.pose.orientation.x;
  odom_msg.pose.pose.orientation.y = odom.pose.orientation.y;
  odom_msg.pose.pose.orientation.z = odom.pose.orientation.z;
  odom_msg.pose.pose.orientation.w = odom.pose.orientation.w;

  for (int i = 0; i < 36; i++) {
    odom_msg.pose.covariance[i] = odom.pose.covariance[i];
  }

  odom_msg.twist.twist.linear.x = odom.twist.linear.x;
  odom_msg.twist.twist.linear.y = odom.twist.linear.y;
  odom_msg.twist.twist.linear.z = odom.twist.linear.z;

  odom_msg.twist.twist.angular.x = odom.twist.angular.x;
  odom_msg.twist.twist.angular.y = odom.twist.angular.y;
  odom_msg.twist.twist.angular.z = odom.twist.angular.z;

  for (int i = 0; i < 36; i++) {
    odom_msg.twist.covariance[i] = odom.twist.covariance[i];
  }

  return odom_msg;
}

sensor_msgs::PointCloud2 to_ros_msg(const PointCloud2 &pointcloud) {
  sensor_msgs::PointCloud2 pointcloud_msg;
  pointcloud_msg.header.stamp =
      ros::Time(pointcloud.timestamp_sec, pointcloud.timestamp_nsec);
  pointcloud_msg.header.frame_id = pointcloud.frame_id;
  pointcloud_msg.height = pointcloud.height;
  pointcloud_msg.width = pointcloud.width;
  pointcloud_msg.fields.resize(pointcloud.fields.size());
  for (size_t i = 0; i < pointcloud_msg.fields.size(); i++) {
    pointcloud_msg.fields[i].name = pointcloud.fields[i].name;
    pointcloud_msg.fields[i].offset = pointcloud.fields[i].offset;
    pointcloud_msg.fields[i].datatype = pointcloud.fields[i].datatype;
    pointcloud_msg.fields[i].count = pointcloud.fields[i].count;
  }

  pointcloud_msg.is_bigendian = pointcloud.is_bigendian;
  pointcloud_msg.point_step = pointcloud.point_step;
  pointcloud_msg.row_step = pointcloud.row_step;
  pointcloud_msg.data = pointcloud.data;
  pointcloud_msg.is_dense = pointcloud.is_dense;

  return pointcloud_msg;
}

}  // namespace sbox_msgs
