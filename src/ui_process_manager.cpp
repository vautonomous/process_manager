#include "ui_process_manager/ui_process_manager.hpp"

UIProcessManager::UIProcessManager(const std::string &node_name, const rclcpp::NodeOptions &options) : Node(node_name,
                                                                                                            options) {

}

void UIProcessManager::shutdownPC(int &result) {
    bp::child c(
            bp::search_path("bash"),
            std::vector<std::string>{
                    "-c", "shutdown"});
    c.wait();
    result = c.exit_code();
}

void UIProcessManager::rebootPC(int &result) {
    bp::child c(
            bp::search_path("bash"),
            std::vector<std::string>{
                    "-c", "reboot"});
    c.wait();
    result = c.exit_code();
}



