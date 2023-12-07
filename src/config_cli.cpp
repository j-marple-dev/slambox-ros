/// @file
/// @author Jongkuk Lim <limjk@jmarple.ai>
/// @copyright 2023 J.Marple
/// @brief SLAMBOX CLI configurator application

#include <cli/cli.h>
#include <cli/clilocalsession.h>
#include <cli/filehistorystorage.h>
#include <cli/loopscheduler.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "applications/config_cli_app.hpp"

/// @brief Print application usage
/// @param argv arguments
void print_usage(char **argv) {
  std::cout << "Usage 1: " << argv[0] << " <config_file>" << std::endl;
  std::cout << "     ex) " << argv[0]
            << " slambox_ros_driver/config/client.yaml\n"
            << std::endl;
  std::cout << "Usage 2: " << argv[0] << " <serial_port> <baudrate>"
            << std::endl;
  std::cout << "     ex) " << argv[0] << " /dev/ttyUSB0 1500000" << std::endl;
}

/// @brief Process arguments and set port name and baudrate
/// @param argc argument count
/// @param argv argument values
/// @param port_name port name to be set
/// @param baudrate baudrate to be set
/// @return 0 if success, otherwise 1
/// @note If argc is 2, then argv[1] is config file path
/// @note If argc is 3, then argv[1] is serial port name and argv[2] is baudrate
/// @note `port_name` and `baudrate` are inplace modified
int handle_arguments(int argc, char **argv, std::string *port_name,
                     int *baudrate) {
  if (argc == 2) {
    YAML::Node client_config;
    try {
      client_config = YAML::LoadFile(argv[1]);
    } catch (const YAML::BadFile &e) {
      std::cout << "Error reading " << argv[1] << std::endl;
      std::cout << "Reason: " << e.what() << std::endl;

      print_usage(argv);
      return 1;
    }

    try {
      *port_name =
          client_config["serial_communication"]["port_name"].as<std::string>();
      *baudrate = client_config["serial_communication"]["baudrate"].as<int>();
    } catch (const YAML::Exception &e) {
      std::cout << "Error parsing " << argv[1] << std::endl;
      std::cout << "Reason: " << e.what() << std::endl;

      print_usage(argv);
      return 1;
    }
  } else if (argc == 3) {
    *port_name = argv[1];
    try {
      *baudrate = std::stoi(argv[2]);
    } catch (const std::invalid_argument &e) {
      std::cout << "Error parsing baudrate " << argv[2] << std::endl;
      std::cout << "Reason: " << e.what() << std::endl;

      print_usage(argv);
      return 1;
    }
  }

  return 0;
}

int main(int argc, char **argv) {
  // Handle arguments exception
  if (argc < 2 || argc > 3) {
    print_usage(argv);
    return 1;
  }

  // Parse arguments
  std::string port_name = "/dev/ttyUSB0";
  int baudrate = -1;

  if (handle_arguments(argc, argv, &port_name, &baudrate) != 0) {
    return 1;
  }

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  LOG(INFO) << "port_name: " << port_name << ", baud_rate: " << baudrate;

  try {
    sbox::ConfigCLIApp config_cli_app(port_name, baudrate);
    std::unique_ptr<cli::Menu> root_menu = config_cli_app.create_root_menu();
    cli::Cli cli(
        std::move(root_menu),
        std::make_unique<cli::FileHistoryStorage>(".config_cli_history"));

    cli.EnterAction([](std::ostream &out) { out << "Welcome" << std::endl; });
    cli.ExitAction([](std::ostream &out) { out << "Bye" << std::endl; });

    cli.StdExceptionHandler(
        [](std::ostream &out, const std::string &cmd, const std::exception &e) {
          out << "Exception caught in cli handler: " << e.what()
              << " handling command: " << cmd << ".\n";
        });

    cli::SetColor();
    cli::LoopScheduler scheduler;
    cli::CliLocalTerminalSession local_session(cli, scheduler, std::cout, 200);
    local_session.ExitAction([&scheduler, &config_cli_app](auto &out) {
      out << "Closing App...\n";
      scheduler.Stop();
      config_cli_app.stop();
    });

    config_cli_app.set_session(&local_session);
    config_cli_app.set_loop_scheduler(&scheduler);
    config_cli_app.run();

    if (!config_cli_app.is_running()) {
      std::cerr << "Error: Failed to open serial communication" << std::endl;
      return 1;
    }

    scheduler.Run();

    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "Error: Unknown exception" << std::endl;
    return 1;
  }

  return 1;
}
