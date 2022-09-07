#include "ui_process_manager/ui_process_manager.hpp"

UIProcessManager::UIProcessManager(const std::string &node_name, const rclcpp::NodeOptions &options) : Node(node_name,
                                                                                                            options) {

    // Create publisher and subscriber objects
    pub_diagnostic_ = create_publisher<std_msgs::msg::UInt8>("ui_process_diagnostic", 1);
    sub_process_command_ = create_subscription<std_msgs::msg::UInt8>("ui_process_command", )
    gprocess_autoware_ = std::make_shared<bp::group>();
}

void UIProcessManager::commandCallback(std_msgs::msg::UInt8::SharedPtr msg) {

}

void UIProcessManager::startAutoware(int &result) {


}

void UIProcessManager::killAutoware(int &result) {

}

void UIProcessManager::restartAutoware(int &result) {

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









