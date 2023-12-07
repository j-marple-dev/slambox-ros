/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief SLAMBOX CLI configurator application logic

#ifndef SLAMBOX_ROS_INCLUDE_APPLICATIONS_CONFIG_CLI_APP_HPP_
#define SLAMBOX_ROS_INCLUDE_APPLICATIONS_CONFIG_CLI_APP_HPP_

#include <cli/cli.h>
#include <cli/clilocalsession.h>
#include <cli/filehistorystorage.h>
#include <cli/loopscheduler.h>

#include <memory>
#include <string>
#include <vector>

#include <sbox/communication/sbox_parser.hpp>
#include <sbox/communication/serial_communication.hpp>

namespace sbox {
/// @brief SLAMBOX CLI configurator application logic class
class ConfigCLIApp : public ParsedMessageInterface {
 public:
  /// @brief Constructor
  /// @param serial_port_name serial port name
  /// @param serial_baud_rate serial baud rate
  ConfigCLIApp(const std::string &serial_port_name, int serial_baud_rate);

  /// @brief Destructor
  ~ConfigCLIApp() { this->stop(); }

  /// @brief Create root menu of CLI
  /// @return root menu
  std::unique_ptr<cli::Menu> create_root_menu() {
    std::unique_ptr<cli::Menu> root_menu =
        std::make_unique<cli::Menu>("sbox", "Root menu");
    this->create_root_menu_(root_menu.get());

    return root_menu;
  }

  /// @brief Set loop scheduler from CLI
  /// @param loop_scheduler loop scheduler
  /// @note This is used for posting messages from callback function to loop
  /// scheduler
  void set_loop_scheduler(cli::LoopScheduler *loop_scheduler) {
    loop_scheduler_ = loop_scheduler;
  }

  /// @brief Set session from CLI
  /// @param session session
  void set_session(cli::CliLocalTerminalSession *session) {
    session_ = session;
  }

  /// @brief Run serial communication
  void run() { serial_communication_.run(); }

  /// @brief Stop serial communication
  void stop() {
    serial_communication_.stop();
    is_checking_ping_timeout_ = false;
    if (ping_timeout_thread_.joinable()) {
      ping_timeout_thread_.join();
    }
  }

  /// @brief Check if serial communication is running
  /// @return true if serial communication is running
  bool is_running() { return serial_communication_.is_running(); }

 private:
  /// Receieve mavlink configuration status
  /// @param enabled mavlink communication enabled flag
  /// @param baudrate mavlink baudrate
  void on_response_mavlink_communication_config(bool enabled,
                                                uint32_t baudrate) override;

  /// Receieve serial configuration status
  /// @param enabled serial communication enabled flag
  /// @param baudrate serial baudrate
  void on_response_serial_communication_config(bool enabled,
                                               uint32_t baudrate) override;

  /// Receieve ethernet configuration status
  /// @param enabled ethernet communication enabled flag
  /// @param port ethernet port
  void on_response_ethernet_communication_config(bool enabled,
                                                 uint32_t port) override;

  /// Receieve lidar configuration status
  /// @param lidar_type lidar type
  void on_response_lidar_config(uint8_t lidar_type) override;

  /// Receieve acknowledge
  /// @param requested_mode requested mode
  /// @param status status
  void on_acknowledge(std::array<uint8_t, 2> requested_mode,
                      uint8_t status) override;

  /// @brief Create root menu of CLI
  /// @param root_menu root menu
  /// @note This is used for creating root menu of CLI
  void create_root_menu_(cli::Menu *root_menu);

  /// @brief Show data status
  /// @param out output stream
  void menu_show_status_(std::ostream &out);

  /// @brief Send ping message
  /// @param out output stream
  void menu_send_ping_(std::ostream &out);

  /// @brief Set communication config
  /// @param out output stream
  /// @param arg1 argument 1
  /// @param arg2 argument 2
  void menu_set_(std::ostream &out, const std::string &arg1, int arg2);

  /// @brief Ping timeout check thread
  /// @details This function waits for ping response for 1 second.
  /// If ping response is not received, Acknowledge failed message will be
  /// sent to `serial_parser_`
  void thread_ping_timeout_();

  /// @brief Show CLI prompt
  /// @details This function shows CLI prompt
  void prompt_();

  /// @brief Serial port name
  std::string serial_port_name_;

  /// @brief Serial baud rate
  int serial_baud_rate_;

  /// @brief Parser for serial communication
  sbox::SBoxParser serial_parser_;

  /// @brief Serial communication
  SerialCommunication serial_communication_;

  /// @brief CLI loop scheduler for posting messages from callback function to
  /// loop
  cli::LoopScheduler *loop_scheduler_ = nullptr;

  /// @brief CLI session
  cli::CliLocalTerminalSession *session_ = nullptr;

  /// @brief Ping timeout check thread
  std::thread ping_timeout_thread_;

  /// @brief Ping timeout check flag
  bool is_checking_ping_timeout_ = false;
};
}  // namespace sbox

#endif  // SLAMBOX_ROS_INCLUDE_APPLICATIONS_CONFIG_CLI_APP_HPP_
