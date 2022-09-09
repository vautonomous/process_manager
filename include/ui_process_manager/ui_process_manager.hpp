#ifndef UI_PROCESS_MANAGER_HPP_
#define UI_PROCESS_MANAGER_HPP_

// Include ROS dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

// Include Boost dependencies
#include <boost/process.hpp>
#include <boost/process/extend.hpp>

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
    void uiSignalHandler (int signum)
    {
        std::cout << "Interrupt signal (" << signum << ") received. Publishing error code" << std::endl;
        killAutoware();

    }
    static void signalHandler(int signum){
        process_manager_.uiSignalHandler(signum);
    }
private:
    static UIProcessManager process_manager_;

    bp::group gprocess_autoware_;
    std::vector<bp::child> processes_;

    bool initialized_ = false;

    std::string map_path_;
    std::string vehicle_model_;
    std::string sensor_model_;

    //!< @brief ui process manager diagnostic publisher
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_diagnostic_;
    //!< @brief ui process manager coomand subscriber
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_process_command_;
    /**
    * @brief callback function for commands
    */
    void commandCallback(std_msgs::msg::UInt8::SharedPtr msg);
    /**
    * @brief for shutting PC down
    */
    void startAutoware();

    /**
    * @brief for shutting PC down
    */
    void killAutoware();

    /**
    * @brief for shutting PC down
    */
    void restartAutoware();

    /**
    * @brief for shutting PC down
    */
    void shutdownPC();

    /**
    * @brief for rebooting PC
    */
    void rebootPC();
    /**
    * @brief for publishing process diagnostic
    */
    void publishDiagnostic(uint8_t status);
};

#endif //UI_PROCESS_MANAGER_HPP_
