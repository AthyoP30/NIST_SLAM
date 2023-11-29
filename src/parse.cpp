#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

//#include "your_octree_message_type.hpp"  // Include the actual message type for Octree data
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <AbstractOcTree.h>

// Define a new class that inherits from rclcpp::Node
class OctreeProcessorNode : public rclcpp::Node {
public:
    OctreeProcessorNode(const std::string& node_name, std::shared_ptr<Octomap::Octree> tree, double threshold)
        : Node(node_name), tree(tree), occupancyThreshold(threshold) {
        
        // Create a publisher for PointCloud2 messages
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("Parse_Oct", 1000);

        //creating a subscriber for Octree
        subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_full", 10, std::bind(&OctreeProcessorNode::octreeCallback, this, std::placeholders::_1));          
        }


    void processOctree(octomap::AbstractOcTree* tree ) {
        // Create PointCloud2 messages for occupied and free points
        sensor_msgs::msg::PointCloud2 pcl_msg_occupied;
        sensor_msgs::msg::PointCloud2 pcl_msg_free;

        // Initialize PointCloud2Iterators for both messages
        sensor_msgs::PointCloud2Iterator<float> iterXOccupied(pcl_msg_occupied, "x");
        sensor_msgs::PointCloud2Iterator<float> iterYOcuppied(pcl_msg_occupied, "y");
        sensor_msgs::PointCloud2Iterator<float> iterZOccupied(pcl_msg_occupied, "z");

        sensor_msgs::PointCloud2Iterator<float> iterXFree(pcl_msg_free, "x");
        sensor_msgs::PointCloud2Iterator<float> iterYFree(pcl_msg_free, "y");
        sensor_msgs::PointCloud2Iterator<float> iterZFree(pcl_msg_free, "z");

        for (octomap::OcTree::tree_iterator it = tree->begin_tree(), end = tree->end_tree(); it != end; ++it) {
            if (it->isleaf() == true) {
                double nodeValue = it->getValue();
                if (nodeValue > occupancyThreshold) {
                    *iterXOccupied = it.getX();
                    *iterYOcuppied = it.getY();
                    *iterZOccupied = it.getZ();

                    // Increment the iterators for the occupied point cloud
                    ++iterXOccupied;
                    ++iterYOcuppied;
                    ++iterZOccupied;
                } else {
                    *iterXFree = it.getX();
                    *iterYFree = it.getY();
                    *iterZFree = it.getZ();

                    // Increment the iterators for the free point cloud
                    ++iterXFree;
                    ++iterYFree;
                    ++iterZFree;
                }
            }
        }

        // Publish PointCloud2 messages
        publisher_->publish(pcl_msg_occupied);
        publisher_->publish(pcl_msg_free);
    }

private:
    std::shared_ptr<Octomap::Octree> tree; // The octree
    double occupancyThreshold; // Threshold for occupancy determination

    // ROS 2 publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    //Ros2 subscriber
    rclcpp::Subscription<Octomap::Octree>::SharedPtr subscription_;

    void octreeCallback(const Octomap::Octree::SharedPtr msg) 
    {
        // Process the received Octree data
        octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(&Octomap);
        processOctree(tree);
    }
};

int main(int argc, char **argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    
    // Create a shared pointer to the Node
    double threshold = 0.5; 
    
    auto node = std::make_shared<OctreeProcessorNode>("octree_processor_node", tree, threshold);

    // Process the octree and publish PointCloud2 messages
    //node->processOctree();

    // Spin the ROS 2 node
    rclcpp::spin(node);

    // Shutdown the ROS 2 node
    //rclcpp::shutdown();

    return 0;
}
