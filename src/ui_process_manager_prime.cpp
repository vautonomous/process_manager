#include "ui_process_manager/ui_process_manager_prime.hpp"


UIProcessManagerPrime::UIProcessManagerPrime(const rclcpp::NodeOptions &options)
        : Node("ui_process_manager_prime_node", options) {
    // Declare default parameters
    map_path_ = declare_parameter("map_path", std::string(""));
    vehicle_model_ = declare_parameter("vehicle_model", std::string(""));
    sensor_model_ = declare_parameter("sensor_model", std::string(""));
    // Create publisher objects
    pub_diagnostic_ = create_publisher<std_msgs::msg::UInt8>("out/process_result", 1);
    pub_nuc_command_ = create_publisher<std_msgs::msg::UInt8>(
            "out/process_command", rclcpp::QoS(1).transient_local());
    // Create subscriber objects
    sub_process_command_ = create_subscription<std_msgs::msg::UInt8>(
            "in/process_command", 1,
            std::bind(&UIProcessManagerPrime::commandCallback, this, std::placeholders::_1));
    sub_nuc_diagnostic_ = create_subscription<std_msgs::msg::UInt8>("in/process_result", rclcpp::QoS(1).transient_local(), std::bind(
            &UIProcessManagerPrime::nucDiagnosticCallback,
            this, std::placeholders::_1));

    // Create service clients
    client_emergency_stop_ = create_client<tier4_external_api_msgs::srv::SetEmergency>(
            "/api/autoware/set/emergency", rmw_qos_profile_services_default);
    client_clear_emergency_ = create_client<std_srvs::srv::Trigger>(
            "/system/clear_emergency", rmw_qos_profile_services_default);
    std::cout << "map_path_: " << map_path_ << std::endl;
    std::cout << "vehicle_model_: " << vehicle_model_ << std::endl;
    std::cout << "sensor_model_: " << sensor_model_ << std::endl;

//    startAutoware();
}

void UIProcessManagerPrime::commandCallback(std_msgs::msg::UInt8::SharedPtr msg) {
    if (msg->data == 0) {
        sendCommandToNUC(static_cast<uint8_t>(Command::PC_SHUTDOWN));
        shutdownPC();
    } else if (msg->data == 1) {
        sendCommandToNUC(static_cast<uint8_t>(Command::PC_REBOOT));
        rebootPC();
    } else if (msg->data == 2) {
        startAutoware();
    } else if (msg->data == 3) {
        restartAutoware();
    } else if (msg->data == 4) {
        killAutoware();
    } else if (msg->data == 5) {
        clearEmergency();
    }else {
        RCLCPP_WARN_ONCE(get_logger(), "Invalid command received!");
    }
}

void UIProcessManagerPrime::startAutoware() {

    if (initialized_) {
        return;
    }

    std::string run_autoware_command =
            "source ~/projects/autoware/install/setup.bash "
            "&& source ~/projects/volt_drivers_ws/install/setup.bash "
            "&& ros2 launch autoware_launch isuzu.launch.xml "
            "map_path:=/opt/autoware/maps "
            "vehicle_model:=isuzu_vehicle sensor_model:=isuzu_sensor is_primary:=true";

    std::string run_leo_vcu_command =
            "source /opt/ros/humble/setup.bash "
            "&& source ~/projects/autoware/install/setup.bash "
            "&& ros2 launch leo_vcu_driver leo_vcu_driver.launch.xml";

    // Give required permissions and start monitors
    bp::system(
            bp::search_path("bash"),
            std::vector<std::string>{
                    "-c",
                    "echo asd | sudo -S /home/volt/projects/volt_scripts/system_monitor/system_monitor.sh"});

    // Run vcu driver
    processes_.push_back(bp::child(
            bp::search_path("bash"), std::vector<std::string>{"-c", run_leo_vcu_command},
            gprocess_autoware_));
    std::this_thread::sleep_for(std::chrono::seconds(3));
    // Define processes to run Autoware, run isuzu.launch.xml
    processes_.push_back(bp::child(
            bp::search_path("bash"), std::vector<std::string>{"-c", run_autoware_command},
            gprocess_autoware_));

//        std::string run_leo_vcu_command =
//            "source /opt/ros/humble/setup.bash "
//            "&& ros2 run demo_nodes_cpp talker";
//    processes_.push_back(bp::child(
//            bp::search_path("bash"), std::vector<std::string>{"-c", run_leo_vcu_command},
//            gprocess_autoware_));
    initialized_ = true;
    bool running = true;
    for (auto &process: processes_) {
        // Check if all processes are running
        running = running && process.running();
    }

    // Send initialization command to NUC
    if(is_nuc_running_){
        sendCommandToNUC(static_cast<uint8_t>(Command::RESTART_AUTOWARE));
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }else{
        sendCommandToNUC(static_cast<uint8_t>(Command::START_AUTOWARE));
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    if (running) {
        UIProcessManagerPrime::publishDiagnostic(static_cast<uint8_t>(State::ACTIVATED));
    } else {
        UIProcessManagerPrime::publishDiagnostic(static_cast<uint8_t>(State::DISACTIVED));
        RCLCPP_ERROR_STREAM(get_logger(), "Prime PC initialization failed!");
    }

}

void UIProcessManagerPrime::killAutoware() {

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
    UIProcessManagerPrime::publishDiagnostic(static_cast<uint8_t>(State::DISACTIVED));
    bp::system(
            bp::search_path("bash"),
            std::vector<std::string>{
                    "-c",
                    "echo asd | sudo -S /home/volt/projects/volt_scripts/system_monitor/kill_process.sh"});

}

void UIProcessManagerPrime::restartAutoware() {
    RCLCPP_INFO(get_logger(), "Autoware will be restarted!");
    killAutoware();
    std::this_thread::sleep_for(std::chrono::seconds(15));
    startAutoware();
}

void UIProcessManagerPrime::shutdownPC() {
    // Send shutdown command to NUC
    RCLCPP_INFO(get_logger(), "Primary PC will be shutdown!");
    bp::child c(bp::search_path("bash"), std::vector<std::string>{"-c", "shutdown now"});
    c.wait();
}

void UIProcessManagerPrime::rebootPC() {
    RCLCPP_INFO(get_logger(), "Primary PC will be rebooted!");
    bp::child c(bp::search_path("bash"), std::vector<std::string>{"-c", "reboot"});
    c.wait();
}

void UIProcessManagerPrime::publishDiagnostic(uint8_t status) {
    std_msgs::msg::UInt8 diagnostic_msg;
    diagnostic_msg.data = static_cast<uint8_t>(status);
    pub_diagnostic_->publish(diagnostic_msg);
}

void UIProcessManagerPrime::sendCommandToNUC(uint8_t command_id) {
    std_msgs::msg::UInt8 command_msg;
    command_msg.data = static_cast<uint8_t>(command_id);
    pub_nuc_command_->publish(command_msg);
}

void UIProcessManagerPrime::nucDiagnosticCallback(std_msgs::msg::UInt8::SharedPtr msg) {

    // Lock mutex before writing to is_nuc_running_ variable
    std::lock_guard<std::mutex> guard(mu_);

    if (msg->data == static_cast<uint8_t>(State::DISACTIVED)) {
        is_nuc_running_ = false;
    } else if (msg->data == static_cast<uint8_t>(State::ACTIVATED)) {
        is_nuc_running_ = true;
    }

    // To start Autoware check if NUC is up
    if (!is_nuc_up_) {
        is_nuc_up_ = true;
        startAutoware();
    }
}

UIProcessManagerPrime::~UIProcessManagerPrime() {
    killAutoware();
    rclcpp::shutdown();
}

void UIProcessManagerPrime::clearEmergency() {
    using tier4_external_api_msgs::msg::ResponseStatus;
    using tier4_external_api_msgs::srv::SetEmergency;

    auto request = std::make_shared<SetEmergency::Request>();
    request->emergency = false;
    client_emergency_stop_->async_send_request(
            request, [this](rclcpp::Client<SetEmergency>::SharedFuture result) {
                const auto & response = result.get();
                if (response->status.code == ResponseStatus::SUCCESS) {
                    RCLCPP_INFO(get_logger(), "service succeeded");
                } else {
                    RCLCPP_WARN(get_logger(), "service failed: %s", response->status.message.c_str());
                }
            });

    auto request_clear_system_emergency = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_clear_emergency_->async_send_request(request_clear_system_emergency, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result){
        const auto & response = result.get();
        if (response->success) {
            RCLCPP_INFO(get_logger(), "service succeeded");
        } else {
            RCLCPP_WARN(
                    get_logger(), "service failed: %s", response->message.c_str());
        }
    });
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(UIProcessManagerPrime)

