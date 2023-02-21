#include "ui_process_manager/ui_process_manager_second.hpp"

UIProcessManagerSecond::UIProcessManagerSecond(const rclcpp::NodeOptions &options)
        : Node("ui_process_manager_second_node", options) {
    // Declare default parameters
    map_path_ = declare_parameter("map_path", std::string(""));
    vehicle_model_ = declare_parameter("vehicle_model", std::string(""));
    sensor_model_ = declare_parameter("sensor_model", std::string(""));
    // Create publisher and subscriber objects
    pub_diagnostic_ = create_publisher<std_msgs::msg::UInt8>("out/process_result", rclcpp::QoS(1).transient_local());
    sub_process_command_ = create_subscription<std_msgs::msg::UInt8>(
            "in/process_command", rclcpp::QoS(1).transient_local(),
            std::bind(&UIProcessManagerSecond::commandCallback, this, std::placeholders::_1));
    // Create timer callback to check if prime node is active
    timer_ = this->create_wall_timer(
            std::chrono::seconds (2), std::bind(&UIProcessManagerSecond::timerCallback, this));

    std::cout << "map_path_: " << map_path_ << std::endl;
    std::cout << "vehicle_model_: " << vehicle_model_ << std::endl;
    std::cout << "sensor_model_: " << sensor_model_ << std::endl;

    // Publish current state to trigger prime node
    publishDiagnostic(static_cast<uint8_t>(node_state_));
}

void UIProcessManagerSecond::commandCallback(std_msgs::msg::UInt8::SharedPtr msg) {
    if (msg->data == static_cast<uint8_t>(Command::PC_SHUTDOWN)) {
        shutdownPC();
    } else if (msg->data == static_cast<uint8_t>(Command::PC_REBOOT)) {
        rebootPC();
    } else if (msg->data == static_cast<uint8_t>(Command::START_AUTOWARE)) {
        startAutoware();
    } else if (msg->data == static_cast<uint8_t>(Command::RESTART_AUTOWARE)) {
        restartAutoware();
    } else if (msg->data == static_cast<uint8_t>(Command::KILL_AUTOWARE)) {
        killAutoware();
    } else {
        RCLCPP_WARN_ONCE(get_logger(), "Invalid command received!");
    }
}

void UIProcessManagerSecond::startAutoware() {
    if (initialized_) {
        return;
    }

    // Create running command for isuzu.launch as secondary
    std::string run_autoware_command =
            "source ~/projects/autoware/install/setup.bash "
            "&& ros2 launch autoware_launch isuzu.launch.xml "
            "map_path:=/opt/autoware/maps "
            "vehicle_model:=isuzu_vehicle sensor_model:=isuzu_sensor is_primary:=false";

    // Create running command for component container
    std::string run_container_command =
            "source /opt/ros/humble/setup.bash "
            "&& source ~/projects/autoware/install/setup.bash "
            "&& ros2 launch autoware_launch pointcloud_container.launch.py use_multithread:=true "
            "container_name:=pointcloud_container";

    // Run autoware in nuc
    processes_.push_back(bp::child(
            bp::search_path("bash"), std::vector<std::string>{"-c", run_autoware_command},
            gprocess_autoware_));
    // Run pointcloud container
    processes_.push_back(bp::child(
            bp::search_path("bash"), std::vector<std::string>{"-c", run_container_command},
            gprocess_autoware_));

//    std::string run_leo_vcu_command =
//            "source /opt/ros/humble/setup.bash "
//            "&& ros2 run demo_nodes_cpp listener";
//    processes_.push_back(bp::child(
//            bp::search_path("bash"), std::vector<std::string>{"-c", run_leo_vcu_command},
//            gprocess_autoware_));
    initialized_ = true;
    bool running = true;
    for (auto &process: processes_) {
        // Check if all processes are running
        running = running && process.running();
    }

    if (running) {
        UIProcessManagerSecond::publishDiagnostic(static_cast<uint8_t>(State::ACTIVATED));
        node_state_ = State::ACTIVATED;
        RCLCPP_INFO(get_logger(), "NUC initialized!");

    } else {
        UIProcessManagerSecond::publishDiagnostic(static_cast<uint8_t>(State::DISACTIVED));
        node_state_ = State::DISACTIVED;
        RCLCPP_ERROR_STREAM(get_logger(), "NUC initialization failed!");
    }
}

void UIProcessManagerSecond::killAutoware() {

    if (processes_.empty()) return;

    // Kill processes in the group
    gprocess_autoware_.terminate();

    for (auto &process: processes_) {
        // Wait processes to exit to avoid zombie processes
        std::cout << "Killing process (" << process.id() << ") " << std::endl;
        process.wait();
    }
    processes_.clear();
    initialized_ = false;
    UIProcessManagerSecond::publishDiagnostic(static_cast<uint8_t>(State::DISACTIVED));
    node_state_ = State::DISACTIVED;
}

void UIProcessManagerSecond::restartAutoware() {
    RCLCPP_INFO(get_logger(), "Autoware will be restarted!");
    killAutoware();
    std::this_thread::sleep_for(std::chrono::seconds(15));
    startAutoware();
}

void UIProcessManagerSecond::shutdownPC() {
    // Send shutdown command to NUC
    RCLCPP_INFO(get_logger(), "NUC will be shutdown!");
    bp::child c(bp::search_path("bash"), std::vector<std::string>{"-c", "echo asd | sudo -S shutdown now"});
    c.wait();
}

void UIProcessManagerSecond::rebootPC() {
    RCLCPP_INFO(get_logger(), "NUC will be rebooted!");
    bp::child c(bp::search_path("bash"), std::vector<std::string>{"-c", "echo asd | sudo -S reboot"});
    c.wait();
}

void UIProcessManagerSecond::publishDiagnostic(uint8_t status) {
    std_msgs::msg::UInt8 diagnostic_msg;
    diagnostic_msg.data = static_cast<uint8_t>(status);
    pub_diagnostic_->publish(diagnostic_msg);
}
UIProcessManagerSecond::~UIProcessManagerSecond() {
    killAutoware();
    rclcpp::shutdown();
}

void UIProcessManagerSecond::timerCallback() {

    if(sub_process_command_->get_publisher_count() < 1 && node_state_ == State::ACTIVATED){
        killAutoware();
    }

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(UIProcessManagerSecond)


