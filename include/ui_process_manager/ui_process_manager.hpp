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

namespace bp = boost::process;
namespace proc_ex = bp::extend;

class UIProcessManager : public rclcpp::Node {
public:
    UIProcessManager(const std::string &node_name, const rclcpp::NodeOptions &options);

private:

    std::shared_ptr<bp::group> gprocess_autoware_;
    std::vector<bp::child> vector_process_;

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
    void startAutoware(int &result);

    /**
    * @brief for shutting PC down
    */
    void killAutoware(int &result);

    /**
    * @brief for shutting PC down
    */
    void restartAutoware(int &result);

    /**
    * @brief for shutting PC down
    */
    void shutdownPC(int &result);

    /**
    * @brief for rebooting PC
    */
    void rebootPC(int &result);
};

#endif //UI_PROCESS_MANAGER_HPP_
