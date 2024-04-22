// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>
#include <fstream>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

// Struct defining the RRT_Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct RRT_Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} RRT_Node;

inline double norm(RRT_Node a, RRT_Node b){
    return pow( pow(a.x - b.x, 2) + pow(a.y + b.y, 2), 0.5 );
}

typedef struct WayPoint{

    double x;
    double y;

    // Function to calculate the distance to another Waypoint
    double distanceTo(WayPoint& other) const {
        return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
    }
} Waypoint;


typedef struct Vec2{
    double x, y;
    Vec2(float x, float y) : x(x), y(y) {}
} Vec2;

inline void parse_wps(const std::string& fname, vector<WayPoint>& wps, visualization_msgs::msg::MarkerArray& wps_viz);

class RRT : public rclcpp::Node {
public:
    RRT();
    virtual ~RRT();
private:

    // TODO: add the publishers and subscribers you need
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    nav_msgs::msg::OccupancyGrid occ_grid;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wps_pub_;
    visualization_msgs::msg::MarkerArray wps_marker;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // hyperparams
    double map_x = 300.0; // width of occ 
    double map_y = 100.0; // beight of occ
    double map_resolution = 0.01;
    const int total_boxes = std::floor( ( map_x * map_y ) ); 

    double map_wid_px = std::floor(map_x / 2.);
    double map_wid_m = map_wid_px * map_resolution;

    int dilation = 25;

    int num_nodes = 800;
    double max_expansion_dist = 0.5;
    
    double goal_threshold = 0.2;

    vector<WayPoint> wps;

    // hyperparams
    float P = 0.4; // p gain
    float clip_val = M_PI / 2;
    float speed = 0.6;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<double> x_dist;
    std::uniform_real_distribution<double> y_dist;

    // callbacks
    // where rrt actually happens
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point);
    RRT_Node steer(RRT_Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(RRT_Node &nearest_node, RRT_Node &new_node);
    bool is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y);
    std::vector<RRT_Node> find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<RRT_Node> &tree, RRT_Node &node);
    double line_cost(RRT_Node &n1, RRT_Node &n2);
    std::vector<int> near(std::vector<RRT_Node> &tree, RRT_Node &node);

    //personal adds
    void publish_path(const std::vector<RRT_Node>& path );
    void publish_goal(const double& goal_x, const double& goal_y );
    void publish_tree(const std::vector<RRT_Node>& path, const std::vector<RRT_Node>& tree );
    Waypoint find_goal(const std::vector<Waypoint>& wps, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg);
    void pure_pursuit(const std::vector<RRT_Node>& path );

};

