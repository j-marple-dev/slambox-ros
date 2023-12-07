/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief Test for converting ROS message into byte array

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <random>

#include <sbox/sbox_msgs/Odometry.hpp>
#include <sbox/sbox_msgs/PointCloud2.hpp>

#include "utils/ros_msg_converter.hpp"

TEST(MSGConverter, OdometryConversion) {
  std::random_device rd;
  std::mt19937 gen(rd());

  ros::Time::init();

  std::uniform_int_distribution<uint32_t> dis_uint32(0, 4294967295);
  std::uniform_int_distribution<uint32_t> dis_nsec(0, 999999999);
  std::uniform_real_distribution<float> dis_float(-1000.0, 1000.0);

  for (int i = 0; i < 1000; i++) {
    sbox_msgs::Odometry odom;
    odom.timestamp_sec = dis_uint32(gen);
    odom.timestamp_nsec = dis_nsec(gen);

    odom.pose.position.x = dis_float(gen);
    odom.pose.position.y = dis_float(gen);
    odom.pose.position.z = dis_float(gen);

    odom.pose.orientation.x = dis_float(gen);
    odom.pose.orientation.y = dis_float(gen);
    odom.pose.orientation.z = dis_float(gen);
    odom.pose.orientation.w = dis_float(gen);

    for (int i = 0; i < 36; i++) {
      odom.pose.covariance[i] = dis_float(gen);
    }

    odom.twist.linear.x = dis_float(gen);
    odom.twist.linear.y = dis_float(gen);
    odom.twist.linear.z = dis_float(gen);

    odom.twist.angular.x = dis_float(gen);
    odom.twist.angular.y = dis_float(gen);
    odom.twist.angular.z = dis_float(gen);

    for (int i = 0; i < 36; i++) {
      odom.twist.covariance[i] = dis_float(gen);
    }

    nav_msgs::Odometry ros_odom = sbox_msgs::to_ros_msg(odom);

    EXPECT_EQ(odom.timestamp_sec, ros_odom.header.stamp.sec);
    EXPECT_EQ(odom.timestamp_nsec, ros_odom.header.stamp.nsec);

    EXPECT_EQ(odom.pose.position.x, ros_odom.pose.pose.position.x);
    EXPECT_EQ(odom.pose.position.y, ros_odom.pose.pose.position.y);
    EXPECT_EQ(odom.pose.position.z, ros_odom.pose.pose.position.z);

    EXPECT_EQ(odom.pose.orientation.x, ros_odom.pose.pose.orientation.x);
    EXPECT_EQ(odom.pose.orientation.y, ros_odom.pose.pose.orientation.y);
    EXPECT_EQ(odom.pose.orientation.z, ros_odom.pose.pose.orientation.z);
    EXPECT_EQ(odom.pose.orientation.w, ros_odom.pose.pose.orientation.w);
    for (int i = 0; i < 36; i++) {
      EXPECT_EQ(odom.pose.covariance[i], ros_odom.pose.covariance[i]);
    }

    EXPECT_EQ(odom.twist.linear.x, ros_odom.twist.twist.linear.x);
    EXPECT_EQ(odom.twist.linear.y, ros_odom.twist.twist.linear.y);
    EXPECT_EQ(odom.twist.linear.z, ros_odom.twist.twist.linear.z);

    EXPECT_EQ(odom.twist.angular.x, ros_odom.twist.twist.angular.x);
    EXPECT_EQ(odom.twist.angular.y, ros_odom.twist.twist.angular.y);
    EXPECT_EQ(odom.twist.angular.z, ros_odom.twist.twist.angular.z);
    for (int i = 0; i < 36; i++) {
      EXPECT_EQ(odom.twist.covariance[i], ros_odom.twist.covariance[i]);
    }
  }
}

TEST(MSGConverter, PointCloud2Conversion) {
  std::random_device rd;
  std::mt19937 gen(rd());

  ros::Time::init();

  std::uniform_int_distribution<uint32_t> dis_uint32(0, 4294967295);
  std::uniform_int_distribution<uint32_t> dis_nsec(0, 999999999);
  std::uniform_real_distribution<float> dis_float(-1000.0, 1000.0);
  std::uniform_int_distribution<uint8_t> dis_uint8(0, 255);

  std::uniform_int_distribution<uint8_t> dis_char(33, 126);
  std::uniform_int_distribution<uint8_t> dis_length(1, 100);
  std::uniform_int_distribution<uint8_t> dis_bool(0, 1);

  for (int i = 0; i < 1000; i++) {
    sbox_msgs::PointCloud2 pointcloud;
    pointcloud.timestamp_sec = dis_uint32(gen);
    pointcloud.timestamp_nsec = dis_nsec(gen);
    pointcloud.frame_id = "";
    for (int j = 0; j < dis_length(gen); j++) {
      pointcloud.frame_id += dis_char(gen);
    }
    pointcloud.height = dis_uint32(gen);
    pointcloud.width = dis_uint32(gen);

    pointcloud.fields.resize(dis_length(gen));
    for (int j = 0; j < pointcloud.fields.size(); j++) {
      pointcloud.fields[j].name = "";
      for (int k = 0; k < dis_length(gen); k++) {
        pointcloud.fields[j].name += dis_char(gen);
      }
      pointcloud.fields[j].offset = dis_uint32(gen);
      pointcloud.fields[j].datatype = dis_uint32(gen);
      pointcloud.fields[j].count = dis_uint32(gen);
    }

    pointcloud.is_bigendian = dis_bool(gen);
    pointcloud.point_step = dis_uint32(gen);
    pointcloud.row_step = dis_uint32(gen);

    pointcloud.data.resize(dis_length(gen));
    for (int j = 0; j < pointcloud.data.size(); j++) {
      pointcloud.data[j] = dis_uint8(gen);
    }
    pointcloud.is_dense = dis_bool(gen);

    sensor_msgs::PointCloud2 ros_pointcloud = sbox_msgs::to_ros_msg(pointcloud);

    EXPECT_EQ(pointcloud.timestamp_sec, ros_pointcloud.header.stamp.sec);
    EXPECT_EQ(pointcloud.timestamp_nsec, ros_pointcloud.header.stamp.nsec);
    EXPECT_EQ(pointcloud.frame_id, ros_pointcloud.header.frame_id);
    EXPECT_EQ(pointcloud.height, ros_pointcloud.height);
    EXPECT_EQ(pointcloud.width, ros_pointcloud.width);

    EXPECT_EQ(pointcloud.fields.size(), ros_pointcloud.fields.size());
    for (int j = 0; j < pointcloud.fields.size(); j++) {
      EXPECT_EQ(pointcloud.fields[j].name, ros_pointcloud.fields[j].name);
      EXPECT_EQ(pointcloud.fields[j].offset, ros_pointcloud.fields[j].offset);
      EXPECT_EQ(pointcloud.fields[j].datatype,
                ros_pointcloud.fields[j].datatype);
      EXPECT_EQ(pointcloud.fields[j].count, ros_pointcloud.fields[j].count);
    }

    EXPECT_EQ(pointcloud.is_bigendian, ros_pointcloud.is_bigendian);
    EXPECT_EQ(pointcloud.point_step, ros_pointcloud.point_step);
    EXPECT_EQ(pointcloud.row_step, ros_pointcloud.row_step);

    EXPECT_EQ(pointcloud.data.size(), ros_pointcloud.data.size());
    for (int j = 0; j < pointcloud.data.size(); j++) {
      EXPECT_EQ(pointcloud.data[j], ros_pointcloud.data[j]);
    }
    EXPECT_EQ(pointcloud.is_dense, ros_pointcloud.is_dense);
  }
}
