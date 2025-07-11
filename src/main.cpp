#include "rclcpp/rclcpp.hpp"
#include "global_map/global_map.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // Create a node for the global map
    auto node = std::make_shared<global_map::OctomapMerger>();
    
    // Spin the node to keep it alive and processing callbacks
    rclcpp::spin(node);
    
    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    
    return 0;
}