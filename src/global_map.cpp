# include "global_map/global_map.hpp"


namespace global_map {

OctomapMerger::OctomapMerger() : Node("gloabl_map_node") {
    // Initialize the global octree with a resolution of 0.1 meters
    global_octree_ = std::make_shared<octomap::OcTree>(0.05);

    // Create a subscriber to listen for incoming octomap messages
    octomap_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "octomap_binary", 10, std::bind(&OctomapMerger::octomapCallback, this, std::placeholders::_1));   
    // Create a publisher to publish the merged octomap
    octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>(
        "merged_octomap", 10); 

    RCLCPP_INFO(this->get_logger(), "OctomapMerger node initialized.");
}


void OctomapMerger::mergeOctomaps(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    // Convert the incoming octomap message to an octree
    octomap::AbstractOcTree* new_octree = octomap_msgs::msgToMap(*msg);

    if (new_octree) {
        // Merge the new octree into the global octree
        octomap::OcTree* local_octree = dynamic_cast<octomap::OcTree*>(new_octree);
        if (local_octree){
            for (auto it = local_octree->begin(); it != local_octree->end(); ++it) {
                // Insert each occupied node from the local octree into the global octree
                if (local_octree->isNodeOccupied(*it)) {
                    global_octree_->updateNode(it.getKey(), true);
                }   
            }
        }
        delete local_octree; // Clean up the temporary octree
        RCLCPP_INFO(this->get_logger(), "Merged new octomap into global octree.");
    }else {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert octomap message to octree.");
    }
}

void OctomapMerger::publishMergedOctomap() {
    // Convert the global octree to an octomap message
    octomap_msgs::msg::Octomap msg;
    
    msg.id = "merged_octomap";
    msg.resolution = global_octree_->getResolution();
    msg.header.frame_id = "world"; // Set the frame_id for the message
    msg.header.stamp = this->now(); // Set the current time as the timestamp
    octomap_msgs::binaryMapToMsg(*global_octree_, msg);

    // Publish the merged octomap message
    octomap_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published merged octomap.");
}

void OctomapMerger::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    // Process the incoming octomap message
    RCLCPP_INFO(this->get_logger(), "Received octomap message with id: %s", msg->id.c_str());

    // Merge the received octomap into the global octree
    mergeOctomaps(msg);

    // Publish the updated global octomap
    publishMergedOctomap();
}




























}