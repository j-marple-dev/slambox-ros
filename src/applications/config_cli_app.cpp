/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief SLAMBOX CLI configurator application logic

#include "applications/config_cli_app.hpp"

#include <memory>
#include <sbox/protocol/acknowledge/acknowledge_protocol.hpp>
#include <sbox/protocol/acknowledge/ping_protocol.hpp>
#include <sbox/protocol/base_protocol.hpp>
#include <sbox/protocol/request/request_ethernet_communication_config.hpp>
#include <sbox/protocol/request/request_lidar_config.hpp>
#include <sbox/protocol/request/request_mavlink_communication_config.hpp>
#include <sbox/protocol/request/request_serial_communication_config.hpp>
#include <sbox/protocol/set/set_ethernet_communication_config.hpp>
#include <sbox/protocol/set/set_lidar_config.hpp>
#include <sbox/protocol/set/set_mavlink_communication_config.hpp>
#include <sbox/protocol/set/set_serial_communication_config.hpp>

namespace sbox {

ConfigCLIApp::ConfigCLIApp(const std::string &serial_port_name,
                           int serial_baud_rate)
    : serial_port_name_(serial_port_name),
      serial_baud_rate_(serial_baud_rate),
      serial_parser_(),
      serial_communication_(serial_port_name_, serial_baud_rate_) {
  serial_parser_.add_parsed_message_callback(this);
  serial_communication_.set_callback(&SBoxParser::parse, &serial_parser_);
}

// cppcheck-suppress unusedFunction
void ConfigCLIApp::on_response_mavlink_communication_config(bool enabled,
                                                            uint32_t baudrate) {
  session_->OutStream() << "[mavlink] " << (enabled ? "Enabled" : "Disabled")
                        << ", baudrate: " << baudrate << std::endl;
  this->prompt_();
}

// cppcheck-suppress unusedFunction
void ConfigCLIApp::on_response_serial_communication_config(bool enabled,
                                                           uint32_t baudrate) {
  session_->OutStream() << "[serial] " << (enabled ? "Enabled" : "Disabled")
                        << ", baudrate: " << baudrate << std::endl;
  this->prompt_();
}
// cppcheck-suppress unusedFunction
void ConfigCLIApp::on_response_ethernet_communication_config(bool enabled,
                                                             uint32_t port) {
  session_->OutStream() << "[ethernet] " << (enabled ? "Enabled" : "Disabled")
                        << ", port: " << port << std::endl;
  this->prompt_();
}

// cppcheck-suppress unusedFunction
void ConfigCLIApp::on_response_lidar_config(uint8_t type) {
  if (protocol::kLidarCodeToType.find(type) !=
      protocol::kLidarCodeToType.end()) {
    session_->OutStream() << "[lidar] " << protocol::kLidarCodeToType.at(type)
                          << std::endl;
  } else {
    session_->OutStream() << "[lidar] Unknown" << std::endl;
  }
  this->prompt_();
}

// cppcheck-suppress unusedFunction
void ConfigCLIApp::on_acknowledge(std::array<uint8_t, 2> requested_mode,
                                  uint8_t status) {
  if (requested_mode == protocol::kModeAckPing) {
    this->is_checking_ping_timeout_ = false;

    if (loop_scheduler_ != nullptr) {
      loop_scheduler_->Post([this, &status]() {
        if (status == protocol::kStatusAckSuccess) {
          std::cout << "\nPing success" << std::endl;
        } else {
          std::cout << "\nPing failed" << std::endl;
        }

        this->prompt_();
      });
    }
  } else if (requested_mode == protocol::kModeSetMavlinkCommConfig) {
    if (status == protocol::kStatusAckSuccess) {
      session_->OutStream()
          << "Set mavlink communication config success" << std::endl;
    } else {
      session_->OutStream()
          << "Set mavlink communication config failed" << std::endl;
    }
    this->prompt_();
    serial_communication_.write(
        RequestMavlinkCommunicationConfigProtocol().encapsulate());
  } else if (requested_mode == protocol::kModeSetSerialCommConfig) {
    if (status == protocol::kStatusAckSuccess) {
      session_->OutStream()
          << "Set serial communication config success" << std::endl;
    } else {
      session_->OutStream()
          << "Set serial communication config failed" << std::endl;
    }
    this->prompt_();
    serial_communication_.write(
        RequestSerialCommunicationConfigProtocol().encapsulate());
  } else if (requested_mode == protocol::kModeSetEthernetCommConfig) {
    if (status == protocol::kStatusAckSuccess) {
      session_->OutStream()
          << "Set ethernet communication config success" << std::endl;
    } else {
      session_->OutStream()
          << "Set ethernet communication config failed" << std::endl;
    }
    this->prompt_();
    serial_communication_.write(
        RequestEthernetCommunicationConfigProtocol().encapsulate());
  } else if (requested_mode == protocol::kModeSetLidarConfig) {
    if (status == protocol::kStatusAckSuccess) {
      session_->OutStream() << "Set lidar config success" << std::endl;
    } else {
      session_->OutStream() << "Set lidar config failed" << std::endl;
    }
    this->prompt_();
    serial_communication_.write(RequestLidarConfigProtocol().encapsulate());
  }
}

void ConfigCLIApp::create_root_menu_(cli::Menu *root_menu) {
  root_menu->Insert(
      "status", [this](std::ostream &out) { this->menu_show_status_(out); },
      "Show SLAMBOX device data status");
  root_menu->Insert(
      "ping", [this](std::ostream &out) { this->menu_send_ping_(out); },
      "Send ping to SLAMBOX device");

  std::unique_ptr<cli::Menu> set_menu =
      std::make_unique<cli::Menu>("set", "Set mode");
  set_menu->Insert(
      "mavlink",
      [this](std::ostream &out) {
        serial_communication_.write(
            RequestMavlinkCommunicationConfigProtocol().encapsulate());
      },
      "Get current config of mavlink");
  set_menu->Insert(
      "mavlink",
      [this](std::ostream &out, bool enable) { out << enable << std::endl; },
      "Enable/disable mavlink", {"true, false"});
  set_menu->Insert(
      "mavlink",
      [this](std::ostream &out, int baudrate) { out << baudrate << std::endl; },
      "Set mavlink baudrate", {"Baudrate (int)"});
  set_menu->Insert(
      "mavlink",
      [this](std::ostream &out, bool enable, int baudrate) {
        serial_communication_.write(SetMavlinkCommunicationConfigProtocol(
                                        enable, static_cast<uint32_t>(baudrate))
                                        .encapsulate());
      },
      "Enable/disable mavlink and set baudrate",
      {"true, false", "Baudrate (int)"});

  set_menu->Insert(
      "serial",
      [this](std::ostream &out) {
        serial_communication_.write(
            RequestSerialCommunicationConfigProtocol().encapsulate());
      },
      "Get current config of serial communication");
  set_menu->Insert(
      "serial",
      [this](std::ostream &out, int baudrate) { out << baudrate << std::endl; },
      "Set serial baudrate", {"Baudrate (int)"});
  set_menu->Insert(
      "serial",
      [this](std::ostream &out, bool enable) { out << enable << std::endl; },
      "Enable/disable serial communication", {"true, false"});
  set_menu->Insert(
      "serial",
      [this](std::ostream &out, bool enable, int baudrate) {
        serial_communication_.write(SetSerialCommunicationConfigProtocol(
                                        enable, static_cast<uint32_t>(baudrate))
                                        .encapsulate());
      },
      "Enable/disable serial communication and set baudrate",
      {"true, false", "Baudrate (int)"});

  set_menu->Insert(
      "ethernet",
      [this](std::ostream &out) {
        serial_communication_.write(
            RequestEthernetCommunicationConfigProtocol().encapsulate());
      },
      "Get current config of ethernet communication");
  set_menu->Insert(
      "ethernet",
      [this](std::ostream &out, bool enable) { out << enable << std::endl; },
      "Enable/disable ethernet communication", {"true, false"});
  set_menu->Insert(
      "ethernet",
      [this](std::ostream &out, int port) { out << port << std::endl; },
      "Set ethernet port", {"Port (int)"});
  set_menu->Insert(
      "ethernet",
      [this](std::ostream &out, bool enable, int port) {
        serial_communication_.write(SetEthernetCommunicationConfigProtocol(
                                        enable, static_cast<uint32_t>(port))
                                        .encapsulate());
      },
      "Enable/disable ethernet communication and set port",
      {"true, false", "Port (int)"});

  set_menu->Insert(
      "lidar",
      [this](std::ostream &out) {
        serial_communication_.write(RequestLidarConfigProtocol().encapsulate());
      },
      "Get current config of lidar");
  set_menu->Insert(
      "lidar",
      [this](std::ostream &out, std::string type) {
        serial_communication_.write(SetLidarConfigProtocol(type).encapsulate());
      },
      "Set lidar type", {"Type (string)"});

  root_menu->Insert(std::move(set_menu));

  root_menu->Insert("set",
                    [this](std::ostream &out, const std::string &arg1,
                           int arg2) { this->menu_set_(out, arg1, arg2); },
                    "", {"Mode name"});
}

void ConfigCLIApp::menu_set_(std::ostream &out, const std::string &arg1,
                             int arg2) {
  if (arg1 == "mavlink") {
    out << "Set mavlink" << std::endl;
  } else if (arg1 == "serial") {
    out << "Set serial" << std::endl;
  } else if (arg1 == "ethernet") {
    out << "Set ethernet" << std::endl;
  } else {
    out << "Invalid argument" << std::endl;
  }

  out << arg1 << ": " << arg2 << std::endl;
}

void ConfigCLIApp::menu_show_status_(std::ostream &out) {
  out << "SLAMBOX device data status" << std::endl;

  serial_communication_.write(
      RequestMavlinkCommunicationConfigProtocol().encapsulate());
  serial_communication_.write(
      RequestSerialCommunicationConfigProtocol().encapsulate());
  serial_communication_.write(
      RequestEthernetCommunicationConfigProtocol().encapsulate());
  serial_communication_.write(RequestLidarConfigProtocol().encapsulate());
}

void ConfigCLIApp::menu_send_ping_(std::ostream &out) {
  out << "Send ping" << std::endl;

  PingProtocol ping_protocol;
  this->serial_communication_.write(ping_protocol.encapsulate());

  if (ping_timeout_thread_.joinable()) {
    this->is_checking_ping_timeout_ = false;
    this->ping_timeout_thread_.join();
  }

  this->ping_timeout_thread_ =
      std::thread(&ConfigCLIApp::thread_ping_timeout_, this);
}

void ConfigCLIApp::thread_ping_timeout_() {
  this->is_checking_ping_timeout_ = true;
  for (int i = 0; i < 100; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (!this->is_checking_ping_timeout_) {
      return;
    }
  }

  AcknowledgeProtocol protocol(protocol::kModeAckPing,
                               protocol::kStatusAckFailed);
  this->serial_parser_.parse(protocol.encapsulate());
}

void ConfigCLIApp::prompt_() {
  if (session_ != nullptr) {
    session_->Prompt();
  } else {
    std::cout << "SLAMBOX> ";
  }
}

}  // namespace sbox
