#include "ui_process_manager/ui_process_manager.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<UIProcessManager>("ui_process_manager_node", node_options);

    rclcpp::spin(node);
    return 0;
}
