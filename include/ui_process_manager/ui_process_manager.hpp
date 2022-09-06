#ifndef UI_PROCESS_MANAGER_HPP_
#define UI_PROCESS_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <boost/process.hpp>

namespace bp = boost::process;

class UIProcessManager : public rclcpp::Node {
public:
    UIProcessManager(const std::string &node_name, const rclcpp::NodeOptions &options);

private:

/**
* @brief for shutting PC down
*/
    void shutdownPC(int& result);


/**
* @brief for rebooting PC
*/
    void rebootPC(int& result);
};

#endif //UI_PROCESS_MANAGER_HPP_
