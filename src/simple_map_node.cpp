#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <fstream>
#include <iostream>
#include "json/json.h" // Include jsoncpp

#include <chrono>
#include <std_msgs/msg/string.hpp>
#include "geometry_msgs/msg/point.hpp"

//using Json;

// Define a struct to represent a map node
struct MapNode {
    float x;               // x coordinate of the node
    float y;               // y coordinate of the node
    float occupied_static; // static occupancy probability (permanent obstacles)
    float occupied_observed; // observed occupancy probability (temporary obstacles)
    std::chrono::system_clock::time_point witness_time; // time when the obstacle was witnessed
    float decay_time;      // decay time for observed obstacles
    // Add more fields as needed
     // Constructor
    MapNode(){}

 // Serialize MapNode to JSON
    void to_json(Json::Value& j) const {
        j["x"] = x;
        j["y"] = y;
        j["occupied_static"] = occupied_static;
        j["occupied_observed"] = occupied_observed;
        j["witness_time"] = std::chrono::system_clock::to_time_t(witness_time);
        j["decay_time"] = decay_time;
    }

    // Deserialize MapNode from JSON
    void from_json(const Json::Value& j) {
        x = j["x"].asFloat();
        y = j["y"].asFloat();
        occupied_static = j["occupied_static"].asFloat();
        occupied_observed = j["occupied_observed"].asFloat();
        witness_time = std::chrono::system_clock::from_time_t(j["witness_time"].asInt());
        decay_time = j["decay_time"].asFloat();
    }
};

class SimpleMapPublisher : public rclcpp::Node {
public:
    std::vector<std::vector<MapNode>> map_;
    //std::vector<MapNode> map_;
    SimpleMapPublisher(const rclcpp::NodeOptions& options) : Node("simple_map_publisher", options) {
        // Define map parameters
        map_resolution_ = 0.1;   // meters per cell
        map_width_ = 200;        // total cells in x direction
        map_height_ = 200;       // total cells in y direction
        map_origin_x_ = -10.0;   // x coordinate of the bottom-left corner of the map
        map_origin_y_ = -10.0;   // y coordinate of the bottom-left corner of the map

        // Define decay rate (1 minute)
        decay_rate_ = 1.0 / 60.0; // 1 minute in seconds

        // Initialize the map
        initialize_map();

        // Create a publisher for the occupancy grid map
        occupancy_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

        // Create a publisher for the cost map
        cost_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("cost_map", 10);

         // Subscribe to the "obstacle" topic to receive witnessed obstacles
        obstacle_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "obstacle", 10, std::bind(&SimpleMapPublisher::obstacle_callback, this, std::placeholders::_1));


        // Subscribe to the "map_node_commands" topic to receive commands for map node
        map_node_commands_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "map_node_commands", 10, std::bind(&SimpleMapPublisher::map_node_commands_callback, this, std::placeholders::_1));

        // Publish the maps periodically
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SimpleMapPublisher::publish_maps, this));

        // Load the map from file
        load_map();
    }

private:
    void initialize_map() {
        // Resize the map vector
   map_.resize(map_width_);

    // Initialize each map node
    for (int i = 0; i < map_width_; ++i) {
        // Resize the inner vector
        map_[i].resize(map_height_);

        for (int j = 0; j < map_height_; ++j) {
            float x = map_origin_x_ + i * map_resolution_;
            float y = map_origin_y_ + j * map_resolution_;

            // Access the MapNode at (i, j) and initialize its properties
            map_[i][j].x = x;
            map_[i][j].y = y;
            map_[i][j].occupied_static = 0.0;
            map_[i][j].occupied_observed = 0.0;
            map_[i][j].decay_time = decay_rate_;
            map_[i][j].witness_time = std::chrono::system_clock::now();
        }
    }
    }

    void load_map() {
    // Load the map from file (default or specified by user)
    std::string map_file = "default_map.json"; // Default map file
    // Check if map file argument is provided
    //if (/* check if map file argument is provided */) {
    //    map_file = /* get map file argument */;
    //}
    std::ifstream file(map_file);
    if (file.is_open()) {
        // Read map data from file and update the map
        Json::Value root;
        try {
            file >> root;
            for (int i = 0; i < map_width_; ++i) {
                for (int j = 0; j < map_height_; ++j) {
                    Json::Value mapNodeJson = root[std::to_string(i) + "_" + std::to_string(j)];
                    MapNode mapNode;
                    mapNode.from_json(mapNodeJson);
                    map_[i][j] = mapNode;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Map loaded from file: %s", map_file.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading map from file: %s", e.what());
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Map file not found. Creating a new map.");
    }
}

void save_map() {
    // Save the map to a file
    // Save the map to a file
    std::string map_file = "default_map.json"; // Default map file
    // Save map data to file
    Json::Value root;
    for (int i = 0; i < map_width_; ++i) {
        for (int j = 0; j < map_height_; ++j) {
            Json::Value mapNodeJson;
            map_[i][j].to_json(mapNodeJson);
            root[std::to_string(i) + "_" + std::to_string(j)] = mapNodeJson;
        }
    }
    std::ofstream file(map_file);
    if (file.is_open()) {
        try {
            file << root;
            file.close();
            RCLCPP_INFO(this->get_logger(), "Map saved to file: %s", map_file.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving map to file: %s", e.what());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open map file for writing: %s", map_file.c_str());
    }
}


    void obstacle_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    if (msg) {
        // Update map nodes with observed obstacles
        auto current_time = std::chrono::system_clock::now();
        // Extract x, y, and z coordinates from the message
        float x = msg->x;
        float y = msg->y;
        float z = msg->z;
        // Calculate index in the map
        int index = static_cast<int>((x - map_origin_x_) / map_resolution_) * map_width_ +
                    static_cast<int>((y - map_origin_y_) / map_resolution_);
        // Check if the index is within bounds of the map
        if (index >= 0 && index < map_width_ * map_height_) {
            // Update observed occupancy probability
            map_[index / map_width_][index % map_width_].occupied_observed = 1.0; // Set observed occupancy to 1.0 (fully occupied)
            map_[index / map_width_][index % map_width_].witness_time = current_time; // Update witness time
        }
    } else {
        // Handle case when msg is null
    }
}

    void map_node_commands_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Process commands for map node
        std::string command = msg->data;
        if (command == "save_map") {
            // Save the map to a file
            save_map();
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
        }
    }

    void publish_maps() {
        // Remove items that have passed the decay time
        auto current_time = std::chrono::system_clock::now();
        for (auto& row : map_) {
            for (auto& node : row) {
                if (std::chrono::duration_cast<std::chrono::seconds>(current_time - node.witness_time).count() > node.decay_time) {
                    // Reset observed occupancy and witness time
                    node.occupied_observed = 0.0;
                    node.witness_time = current_time;
                }
            }
        }

        // Create and populate the occupancy grid map message
        nav_msgs::msg::OccupancyGrid occupancy_map_msg;
        occupancy_map_msg.header.stamp = this->now();
        occupancy_map_msg.header.frame_id = "map";
        occupancy_map_msg.info.resolution = map_resolution_;
        occupancy_map_msg.info.width = map_width_;
        occupancy_map_msg.info.height = map_height_;
        occupancy_map_msg.info.origin.position.x = map_origin_x_;
        occupancy_map_msg.info.origin.position.y = map_origin_y_;
        occupancy_map_msg.info.origin.position.z = 0.0;
        occupancy_map_msg.info.origin.orientation.x = 0.0;
        occupancy_map_msg.info.origin.orientation.y = 0.0;
        occupancy_map_msg.info.origin.orientation.z = 0.0;
        occupancy_map_msg.info.origin.orientation.w = 1.0;

        // Populate the occupancy grid map data
        occupancy_map_msg.data.resize(map_width_ * map_height_);
        for (int i = 0; i < map_width_; ++i) {
            // for (int j = 0; j < map_height_; ++j) {
            //     // Convert map node data to occupancy grid data (for example, convert float occupancy to int)
            //     // For simplicity, let's assume occupancy is between 0 and 100
            //     int occupancy = static_cast<int>((map_[i * map_width_ + j].occupied_static +
            //                                        map_[i * map_width_ + j].occupied_observed) * 100);
            //     // Clamp occupancy values to be within [0, 100]
            //     occupancy = std::max(0, std::min(100, occupancy));
            //     occupancy_map_msg.data[i * map_width_ + j] = occupancy;
            // }
        }

        // Publish the occupancy grid map
        occupancy_publisher_->publish(occupancy_map_msg);

        // Create and populate the cost map message
        nav_msgs::msg::OccupancyGrid cost_map_msg = occupancy_map_msg; // Copy occupancy grid metadata
        // Populate the cost map data based on occupancy probability (dummy example)
        for (int i = 0; i < map_width_; ++i) {
            for (int j = 0; j < map_height_; ++j) {
                // if (map_[i * map_width_ + j].occupied_static > 0.5 || map_[i * map_width_ + j].occupied_observed > 0.5) {
                //     cost_map_msg.data[i * map_width_ + j] = 100; // occupied space
                // } else {
                //     cost_map_msg.data[i * map_width_ + j] = 0; // free space
                // }
            }
        }

        // Publish the cost map
        cost_publisher_->publish(cost_map_msg);
    }

    // Map parameters
    float map_resolution_;
    int map_width_;
    int map_height_;
    float map_origin_x_;
    float map_origin_y_;

    // Vector to store map nodes
    //std::vector<MapNode> map_;

    // ROS publishers, subscriptions, and timer
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr obstacle_subscription_;    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_node_commands_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Decay rate (in seconds)
    float decay_rate_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    //rclcpp::spin(std::std::make_shared<SimpleMapPublisher>());
    rclcpp::spin(std::make_shared<SimpleMapPublisher>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
