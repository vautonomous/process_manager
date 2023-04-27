#ifndef UI_PROCESS_MANAGER_PRIME_HPP_
#define UI_PROCESS_MANAGER_PRIME_HPP_

// Include ROS dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

// Include Boost dependencies
#include <boost/process.hpp>
#include <boost/process/extend.hpp>
#include <boost/asio/io_context.hpp>

// Include others
#include <vector>
#include <string>
#include <csignal>
#include <iostream>
#include <mutex>

// Include Autoware dependencies

#include <tier4_external_api_msgs/srv/set_emergency.hpp>

namespace bp = boost::process;
namespace proc_ex = bp::extend;

class UIProcessManagerPrime : public rclcpp::Node {
public:
    UIProcessManagerPrime(const rclcpp::NodeOptions &options);
  ~UIProcessManagerPrime() override;
  void killAutoware();

private:

    enum class State {
        DISACTIVED,
        ACTIVATED
    };
    enum class Command {
        PC_SHUTDOWN,
        PC_REBOOT,
        START_AUTOWARE,
        RESTART_AUTOWARE,
        KILL_AUTOWARE
    };

    bp::group gprocess_autoware_;
    std::vector<bp::child> processes_;

    bool initialized_ = false;
    bool is_nuc_running_ = false;
    bool is_nuc_up_ = false;
    bool current_external_emergency_{false};
    bool current_emergency_state{false};

    std::string map_path_;
    std::string vehicle_model_;
    std::string sensor_model_;

    std::mutex mu_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_diagnostic_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_nuc_command_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_process_command_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_nuc_diagnostic_;

    rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedPtr client_emergency_stop_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_clear_emergency_;

    void commandCallback(std_msgs::msg::UInt8::SharedPtr msg);

    void nucDiagnosticCallback(std_msgs::msg::UInt8::SharedPtr msg);

    void startAutoware();

    void restartAutoware();

    void shutdownPC();

    void rebootPC();

    void publishDiagnostic(uint8_t status);

    void sendCommandToNUC(uint8_t command_id);

    void clearEmergency();
};

#endif //UI_PROCESS_MANAGER_PRIME_HPP_
