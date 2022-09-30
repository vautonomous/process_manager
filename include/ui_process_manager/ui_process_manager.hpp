#ifndef UI_PROCESS_MANAGER_HPP_
#define UI_PROCESS_MANAGER_HPP_

// Include ROS dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

// Include Boost dependencies
#include <boost/process.hpp>
#include <boost/process/extend.hpp>
#include <boost/asio/io_context.hpp>

// Include others
#include <vector>
#include <string>
#include <csignal>
#include <iostream>

namespace bp = boost::process;
namespace proc_ex = bp::extend;

class UIProcessManager : public rclcpp::Node {
public:
    UIProcessManager(const std::string &node_name, const rclcpp::NodeOptions &options);

    void killAutoware();
private:
    bp::group gprocess_autoware_;
    std::vector<bp::child> processes_;

    bool initialized_ = false;

    std::string map_path_;
    std::string vehicle_model_;
    std::string sensor_model_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_diagnostic_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_process_command_;

    void commandCallback(std_msgs::msg::UInt8::SharedPtr msg);

    void startAutoware();

    void restartAutoware();

    void shutdownPC();

    void rebootPC();

    void publishDiagnostic(uint8_t status);
};

#endif //UI_PROCESS_MANAGER_HPP_
