 #include "ui_process_manager/ui_process_manager.hpp"

UIProcessManager::UIProcessManager(const std::string &node_name, const rclcpp::NodeOptions &options) : Node(node_name,
                                                                                                            options) {
    // Declare default parameters
    map_path_ = declare_parameter("map_path", std::string(""));
    vehicle_model_ = declare_parameter("vehicle_model", std::string(""));
    sensor_model_ = declare_parameter("sensor_model", std::string(""));
    // Create publisher and subscriber objects
    pub_diagnostic_ = create_publisher<std_msgs::msg::UInt8>("out/process_result", 1);
    sub_process_command_ = create_subscription<std_msgs::msg::UInt8>("in/process_command", 1,
                                                                     std::bind(&UIProcessManager::commandCallback, this,
                                                                               std::placeholders::_1));
}

void UIProcessManager::commandCallback(std_msgs::msg::UInt8::SharedPtr msg) {
    if (msg->data == 0) {
        shutdownPC();
    } else if (msg->data == 1) {
        rebootPC();
    } else if (msg->data == 2) {
        startAutoware();
    } else if (msg->data == 3) {
        restartAutoware();
    } else if (msg->data == 4) {
        killAutoware();
    } else {
        RCLCPP_WARN_ONCE(
                get_logger(), "Invalid command received!");
    }

}

void UIProcessManager::startAutoware() {
    if (!initialized_) {
        std::string run_autoware_command =
                "source ~/projects/autoware/install/setup.bash && ros2 launch autoware_launch isuzu.launch.xml map_path:=" +
                map_path_ + " vehicle_model:=" + vehicle_model_ + " sensor_model:=" + sensor_model_;
        std::string run_container_command = "source ~/projects/autoware/install/setup.bash && ros2 launch autoware_launch pointcloud_container.launch.py use_multithread:=true container_name:=pointcloud_container";
        std::string run_camera_command = "source /home/volt/projects/volt_drivers_ws/install/setup.bash && ros2 run arena_camera arena_camera_node_exe --ros-args --params-file /home/volt/projects/volt_drivers_ws/src/arena_camera/param/volt_multi_camera.param.yaml";

        // Give required permissions and start monitors
        bp::system(bp::search_path("bash"),std::vector<std::string>{
                "-c",
                "echo asd | sudo -S /home/volt/projects/volt_scripts/system_monitor/system_monitor.sh"});

        //Define processes to run Autoware, run isuzu.launch.xml
        processes_.push_back(bp::child(bp::search_path("bash"),
                                       std::vector<std::string>{
                                               "-c",
                                               run_autoware_command}, gprocess_autoware_));
        // Run pointcloud container
        processes_.push_back(bp::child(bp::search_path("bash"),
                                       std::vector<std::string>{
                                               "-c",
                                               run_container_command},
                                       gprocess_autoware_));
        // Run camera driver, TEMP
        processes_.push_back(bp::child(bp::search_path("bash"),
                                       std::vector<std::string>{
                                               "-c",
                                               run_camera_command},
                                       gprocess_autoware_));
        initialized_ = true;
        bool running = true;
        for (auto &process: processes_) {
            // Check if all processes are running
            running = running && process.running();
        }
        if (running) {
            UIProcessManager::publishDiagnostic(1);
        } else {
            UIProcessManager::publishDiagnostic(0);
        }
    }

}

void UIProcessManager::killAutoware() {
    if (!processes_.empty()) {
        // Kill processes in the group
        gprocess_autoware_.terminate();
        for (auto &process: processes_) {
            // Wait processes to exit to avoid zombie processes
            std::cout << "Killing process (" << process.id() << ") " << std::endl;
            process.wait();
        }
        processes_.clear();
        initialized_ = false;
        UIProcessManager::publishDiagnostic(0);
        bp::system(bp::search_path("bash"),std::vector<std::string>{
                "-c",
                "echo asd | sudo -S /home/volt/projects/volt_scripts/system_monitor/kill_process.sh"});
    }
}

void UIProcessManager::restartAutoware() {

    killAutoware();
    std::this_thread::sleep_for(std::chrono::seconds(15));
    startAutoware();

}

void UIProcessManager::shutdownPC() {
    bp::child c(
            bp::search_path("bash"),
            std::vector<std::string>{
                    "-c", "shutdown"});
    c.wait();
}

void UIProcessManager::rebootPC() {
    bp::child c(
            bp::search_path("bash"),
            std::vector<std::string>{
                    "-c", "reboot"});
    c.wait();
}

void UIProcessManager::publishDiagnostic(uint8_t status) {
    std_msgs::msg::UInt8 diagnostic_msg;
    diagnostic_msg.data = static_cast<uint8_t>(status);
    pub_diagnostic_->publish(diagnostic_msg);
}










