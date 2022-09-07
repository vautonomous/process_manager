#include "ui_process_manager/ui_process_manager.hpp"

UIProcessManager::UIProcessManager(const std::string &node_name, const rclcpp::NodeOptions &options) : Node(node_name,
                                                                                                            options) {
    // Declare default parameters
    map_path_ = declare_parameter("map_path", std::string("/home/volt/projects/gebze_map"));
    vehicle_model_ = declare_parameter("vehicle_model", std::string("isuzu_vehicle"));
    sensor_model_ = declare_parameter("sensor_model", std::string("isuzu_sensor"));

    // Create publisher and subscriber objects
    pub_diagnostic_ = create_publisher<std_msgs::msg::UInt8>("ui_process_diagnostic", 1);
    sub_process_command_ = create_subscription<std_msgs::msg::UInt8>("ui_process_command", 1,
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

    // Define processes to run Autoware, run isuzu.launch.xml
    //processes_.push_back(bp::child(bp::search_path("bash"),
    //                               std::vector<std::string>{
    //                                       "-c", "ros2 launch autoware_launch isuzu.launch.xml", " ",
    //                                       "map_path:=", map_path_, " ",
    //                                       "vehicle_model:=", vehicle_model_, " ",
    //                                       "sensor_model:=", sensor_model_}, gprocess_autoware_,
    //                               UIProcessManager::status_handler()));
//
    //// Run pointcloud container
    //processes_.push_back(bp::child(bp::search_path("bash"),
    //                               std::vector<std::string>{
    //                                       "-c",
    //                                       "ros2 launch autoware_launch pointcloud_container.launch.py use_multithread:=true container_name:=pointcloud_container"},
    //                               gprocess_autoware_, UIProcessManager::status_handler()));
//
    //// Run camera driver, TEMP
    //processes_.push_back(bp::child(bp::search_path("bash"),
    //                               std::vector<std::string>{
    //                                       "-c",
    //                                       "ros2 run arena_camera arena_camera_node_exe --ros-args --params-file /home/volt/projects/volt_drivers_ws/src/arena_camera/param/volt_multi_camera.param.yaml"},
    //                               gprocess_autoware_, UIProcessManager::status_handler()));
    processes_.push_back(bp::child(bp::search_path("bash"),
                                   std::vector<std::string>{
                                           "-c",
                                           "ros2 launch demo_nodes_cpp talker_listener.launch.py"},
                                   gprocess_autoware_, UIProcessManager::status_handler()));
}

void UIProcessManager::killAutoware() {
    // Kill processes in the group
    gprocess_autoware_.terminate();
    for(auto &process: processes_){
        // Wait processes to exit
        process.wait();
        RCLCPP_WARN_ONCE(
                get_logger(), "Process %d have been killed with exit code %d, status %d", process.id(), process.exit_code());
    }
}

void UIProcessManager::restartAutoware() {

    killAutoware();
    std::this_thread::sleep_for(std::chrono::seconds(10));
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











