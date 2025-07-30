# include "global_map/global_map.hpp"


namespace global_map {

OctomapMerger::OctomapMerger() : Node("gloabl_map_node") {

    // Declare and get parameters
    this->declare_parameter("octomap.resolution", 0.2);
    this->declare_parameter("octomap.prob_hit", 0.7);
    this->declare_parameter("octomap.prob_miss", 0.45);
    this->declare_parameter("octomap.clamping_thres_min", 0.12);
    this->declare_parameter("octomap.clamping_thres_max", 0.97);

    this->get_parameter("octomap.resolution", resolution_);
    this->get_parameter("octomap.prob_hit", prob_hit_);
    this->get_parameter("octomap.prob_miss", prob_miss_);
    this->get_parameter("octomap.clamping_thres_min", clamping_thres_min_);
    this->get_parameter("octomap.clamping_thres_max", clamping_thres_max_);

    // Initialize the global octree with a resolution of 0.1 meters
    global_octree_ = std::make_shared<octomap::OcTree>(resolution_);
    global_octree_->setProbHit(prob_hit_);
    global_octree_->setProbMiss(prob_miss_);
    global_octree_->setClampingThresMin(clamping_thres_min_);
    global_octree_->setClampingThresMax(clamping_thres_max_);

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

    if (!new_octree) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert octomap message to octree.");
        return;
    }

    // cast to OcTree
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(new_octree);
    if (!octree) {
        RCLCPP_ERROR(this->get_logger(), "Failed to cast octree.");
        return;
    }

    //Update the global octree with the new octree
    for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
        octomap::OcTreeKey key = it.getKey();
        float prob = it->getOccupancy();
        global_octree_->setNodeValue(key, prob, false);
    }

    // Update internal nodes
    global_octree_->updateInnerOccupancy();

    RCLCPP_INFO(this->get_logger(), "Merged new octomap into global octree.");

    delete octree;  // Clean up
    
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