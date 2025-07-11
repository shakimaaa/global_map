/**
 * @file global_map.hpp
 * @author Shakima
 * @date 2024-06-09
 * @brief Header file for global map module.
 */

#ifndef GLOBAL_MAP_GLOBAL_MAP_HPP
#define GLOBAL_MAP_GLOBAL_MAP_HPP
#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

namespace global_map {

// TODO: Add class and function declarations here.

class OctomapMerger : public rclcpp::Node {
public:
    OctomapMerger();

private:
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

    void mergeOctomaps(const octomap_msgs::msg::Octomap::SharedPtr msg);

    void publishMergedOctomap();


    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscriber_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;

    std::shared_ptr<octomap::OcTree> global_octree_;




};

} // namespace global_map

#endif // GLOBAL_MAP_GLOBAL_MAP_HPP