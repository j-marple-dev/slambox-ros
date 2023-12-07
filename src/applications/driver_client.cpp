/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief SLAMBOX ROS Driver Client

#include "applications/driver_client.hpp"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <fmt/core.h>

#include <memory>

#include <sbox/communication/serial_communication.hpp>
#include <sbox/protocol/acknowledge/ping_protocol.hpp>
#include <sbox/protocol/base_protocol.hpp>
#include <sbox/protocol/command/command_save_pcd_protocol.hpp>
#include <sbox/protocol/request/request_ethernet_communication_config.hpp>
#include <sbox/protocol/request/request_mavlink_communication_config.hpp>
#include <sbox/protocol/request/request_serial_communication_config.hpp>

#include "utils/ros_msg_converter.hpp"
#include "utils/string_utils.hpp"

namespace sbox {

SLAMBOXDriverClient::SLAMBOXDriverClient(ros::NodeHandle nh,
                                         ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh), serial_parser_(4096), udp_parser_(131070) {
  nh_.param<std::string>("serial_communication/port_name", serial_port_name_,
                         "/dev/ttyUSB1");
  nh_.param<int>("serial_communication/baudrate", serial_baud_rate_, 921600);

  nh_.param<bool>("serial_communication/enabled", is_serial_enabled_, true);
  nh_.param<bool>("ethernet_communication/enabled", is_ethernet_enabled_,
                  false);

  nh_.param<std::string>("ethernet_communication/server_addr", udp_ip_, "");
  nh_.param<int>("ethernet_communication/port", udp_port_, 21580);

  nh_.param<std::string>("publish/odom_topic", publish_odom_topic_,
                         "/SLAMBOX/odom");
  nh_.param<std::string>("publish/pointcloud_topic", publish_pointcloud_topic_,
                         "/SLAMBOX/pointcloud");
  nh_.param<std::string>("subscribe/request_topic", subscribe_request_topic_,
                         "/SLAMBOX/request");

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(publish_odom_topic_, 1);
  pointcloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(publish_pointcloud_topic_, 1);
  request_sub_ = nh_.subscribe(subscribe_request_topic_, 1,
                               &SLAMBOXDriverClient::callback_request_, this);

  if (is_serial_enabled_) {
    serial_communication_ = std::make_unique<SerialCommunication>(
        serial_port_name_, serial_baud_rate_);

    serial_parser_.add_parsed_message_callback(this);
    serial_communication_->set_callback(&SBoxParser::parse, &serial_parser_);
    serial_communication_->run();
  }

  if (is_ethernet_enabled_) {
    udp_communication_ =
        std::make_unique<UDPCommunication>(false, udp_ip_, udp_port_);
    udp_parser_.add_parsed_message_callback(this);
    udp_communication_->set_callback(&SBoxParser::parse, &udp_parser_);
    udp_communication_->run();
  }

  send_ping_thread_ =
      std::thread(&SLAMBOXDriverClient::thread_send_ping_, this);
}

SLAMBOXDriverClient::~SLAMBOXDriverClient() {
  this->is_send_ping_thread_running_ = false;
  send_ping_thread_.join();

  if (serial_communication_ != nullptr) {
    serial_communication_->stop();
  }

  if (udp_communication_ != nullptr) {
    udp_communication_->stop();
  }
}

void SLAMBOXDriverClient::thread_send_ping_() {
  while (ros::ok() && this->is_send_ping_thread_running_) {
    if (is_serial_enabled_) {
      if (!this->is_server_alive()) {
        serial_communication_->set_baudrate(115200);
        serial_communication_->write(PingProtocol().encapsulate());
        serial_communication_->set_baudrate(this->serial_baud_rate_);
      }
      serial_communication_->write(PingProtocol().encapsulate());

      LOG(INFO) << "baudrate " << serial_communication_->get_baudrate()
                << std::endl;
    }

    if (is_ethernet_enabled_) {
      udp_communication_->write(PingProtocol().encapsulate());
    }

    if (!is_server_alive()) {
      LOG(WARNING) << "Server is not alive" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

// cppcheck-suppress unusedFunction
void SLAMBOXDriverClient::on_push_odometry(const sbox_msgs::Odometry &odom) {
  nav_msgs::Odometry odom_msg = sbox_msgs::to_ros_msg(odom);
  odom_pub_.publish(odom_msg);
}

// cppcheck-suppress unusedFunction
void SLAMBOXDriverClient::on_push_pointcloud(
    const sbox_msgs::PointCloud2 &pointcloud) {
  sensor_msgs::PointCloud2 pointcloud_msg = sbox_msgs::to_ros_msg(pointcloud);
  pointcloud_pub_.publish(pointcloud_msg);
}

void SLAMBOXDriverClient::on_response_mavlink_communication_config(
    bool enabled, uint32_t baudrate) {
  LOG(INFO) << "[mavlink] " << (enabled ? "Enabled" : "Disabled")
            << ", baudrate: " << baudrate << std::endl;
}

void SLAMBOXDriverClient::on_response_serial_communication_config(
    bool enabled, uint32_t baudrate) {
  LOG(INFO) << "[serial] " << (enabled ? "Enabled" : "Disabled")
            << ", baudrate: " << baudrate << std::endl;
}
void SLAMBOXDriverClient::on_response_ethernet_communication_config(
    bool enabled, uint32_t port) {
  LOG(INFO) << "[ethernet] " << (enabled ? "Enabled" : "Disabled")
            << ", port: " << port << std::endl;
}

void SLAMBOXDriverClient::on_acknowledge(std::array<uint8_t, 2> requested_mode,
                                         uint8_t status) {
  LOG(INFO) << fmt::format(
      "[Acknowledge] Requested mode: 0x{:02X} {:02X},  Status: 0x{:02X}",
      requested_mode[0], requested_mode[1], status);

  if (requested_mode == sbox::protocol::kModeAckPing) {
    last_ping_time_ = ros::Time::now();
  }
}

bool SLAMBOXDriverClient::is_server_alive() {
  if (ros::Time::now() - last_ping_time_ > ros::Duration(3.0)) {
    return false;
  }
  return true;
}

void SLAMBOXDriverClient::callback_request_(const std_msgs::String &msg) {
  std::vector<std::string> split_msg = string_utils::split(msg.data, ',');
  std::vector<uint8_t> data;
  if (split_msg.front() == "GET") {
    if (split_msg[1] == "DATA_STATUS") {
      std::vector<uint8_t> protocol1 =
          RequestMavlinkCommunicationConfigProtocol().encapsulate();
      std::vector<uint8_t> protocol2 =
          RequestSerialCommunicationConfigProtocol().encapsulate();
      std::vector<uint8_t> protocol3 =
          RequestEthernetCommunicationConfigProtocol().encapsulate();
      data.insert(data.end(), protocol1.begin(), protocol1.end());
      data.insert(data.end(), protocol2.begin(), protocol2.end());
      data.insert(data.end(), protocol3.begin(), protocol3.end());
    }

  } else if (split_msg.front() == "CMD") {
    if (split_msg[1] == "SAVE_PCD" && split_msg.size() >= 4) {
      bool save, reset;
      save = (split_msg[2] == "1");
      reset = (split_msg[3] == "1");
      CommandSavePCDProtocol command_save_pcd_protocol(save, reset);
      data = command_save_pcd_protocol.encapsulate();
    }
  }
  if (data.size() < 8) {
    LOG(WARNING) << "Unknown request: " << msg.data;
    return;
  }

  if (is_serial_enabled_) {
    serial_communication_->write(data);
  }

  if (is_ethernet_enabled_) {
    udp_communication_->write(data);
  }
}
}  // namespace sbox
