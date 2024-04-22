// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

#define TREE true

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
// RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()), x_dist(0.0, x_dist), y_dist(-map_wid, map_wid ) {
RRT::RRT()
: rclcpp::Node("rrt_node"), 
  gen(std::random_device{}()), // Correctly initialized with a seed
  x_dist(0.0, map_y * map_resolution),  // Replace max_x_value with the actual maximum bound for x_dist
  y_dist( -map_wid_m, map_wid_m )
   {
    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need

    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "/pf/viz/inferred_pose";
    string scan_topic = "/scan";

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid", 1);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/chosen_path", 1);
    goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_point", 1);
    tree_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tree", 1);
    wps_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints", 1);
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);

    wps_marker = visualization_msgs::msg::MarkerArray();
    parse_wps("/home/moody/f1tenth_ws/waypoints/waypoints_rrt.csv", wps, wps_marker);

    // TODO: create a occupancy grid
    occ_grid = nav_msgs::msg::OccupancyGrid();
    occ_grid.header.frame_id = "ego_racecar/base_link";

    occ_grid.info.height = static_cast<int>(map_x); 
    occ_grid.info.width = static_cast<int>(map_y);
    occ_grid.info.resolution = map_resolution;

    geometry_msgs::msg::Pose origin = geometry_msgs::msg::Pose();
    // origin.position.x = map_resolution;
    origin.position.y = ( - map_x / 2 ) * map_resolution;
    origin.position.z = 0.1;
    this->occ_grid.info.origin = origin;

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    occ_grid.data.assign(map_x * map_y, 0);

    for( size_t i = 0; i < scan_msg->ranges.size(); i++ ){

        float theta = (i * scan_msg->angle_increment + scan_msg->angle_min);

        if(theta < -M_PI/2 || theta > M_PI/2 ) continue;

        double x = std::floor( scan_msg->ranges[i] * std::cos(theta) / map_resolution ) ;
        double y = std::floor( scan_msg->ranges[i] * std::sin(theta) / map_resolution ) ;

        int idx_x = (int) x; //- (int)( std::floor( map_y ) );
        int idx_y = (int) y + (int)( map_wid_px );
        // int idx_y = (int) y + (iint)( std::floor( map_y / 2.) );
        if(idx_x < 0 || idx_x >= map_y || idx_y < 0 || idx_y >= map_x) continue;

        for(int j = -dilation; j <= dilation; j++ ) {
            for(int k = -dilation; k <= dilation; k++) {
                
                int x_idx = idx_x + j;
                int y_idx = idx_y + k;

                // Check boundaries
                if(x_idx < 0 || x_idx >= map_y || y_idx < 0 || y_idx >= map_x) continue;

                int idx = x_idx + y_idx * map_y;
                // if(idx >= 0 && idx < total_boxes) {
                occ_grid.data[idx] = 100;
            }
        }
    }
    
    // TODO: update your occupancy grid
    occ_grid.header.stamp = this->now();
    grid_pub_->publish(occ_grid);

}

Waypoint RRT::find_goal(const std::vector<Waypoint>& wps, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg){

    tf2::Quaternion quat(
        pose_msg->pose.orientation.x,
        pose_msg->pose.orientation.y,
        pose_msg->pose.orientation.z,
        pose_msg->pose.orientation.w
    );
    // Convert geometry_msgs::Quaternion to tf2::Quaternion

    // Convert quaternion to roll, pitch, yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    WayPoint current = {pose_msg->pose.position.x, pose_msg->pose.position.y};

    Waypoint farthest;
    double farthest_dist = -std::numeric_limits<double>::max();

    for( const WayPoint& wp : wps){

        // yaw is the theta we're interested in
        Waypoint goal = {wp.x - current.x, wp.y - current.y};
        
        double x =  goal.y*sin(yaw) + goal.x*cos(yaw);
        double y = -goal.x*sin(yaw) + goal.y*cos(yaw);

        goal.x = x;
        goal.y = -y; // positive y is left

        // ensure x, y is within the correct bounds
        if( x < 0 || x > (map_y * map_resolution)) continue;
        if( y > map_wid_m || y < -map_wid_m ) continue;

        double dist = std::sqrt( x*x + y*y );

        if(dist > farthest_dist){
            farthest_dist = dist;
            farthest = goal;
        }

    }

    return farthest;

}

void RRT::pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    wps_pub_->publish(wps_marker);

    // Waypoint goal = find_goal(wps, pose_msg);
    /*
    Goal is in meters:
    heading of car is x
    side-to-side is y
    */
    Waypoint goal = find_goal(wps, pose_msg);

    // y frame from marker and y frame for planning are swapped, hence -y
    publish_goal( goal.x , -goal.y);

    // tree as std::vector
    std::vector<RRT_Node> tree;

    RRT_Node root_node;
    // new node starts at the center x pos in px centers
    root_node.x = 0;
    root_node.y = 0;
    root_node.cost = 
    root_node.is_root = true;
    tree.push_back(root_node);    

    // TODO: fill in the RRT main loop
    for( int i = 0; i < num_nodes; i++){
        
        // sample a pixel center within the occupancy grid
        std::vector<double> sample_ = sample();
        int nearest_idx = nearest(tree, sample_);
        // publish_goal(sample_[0], sample_[1]);

        RRT_Node new_node = steer(tree[nearest_idx], sample_);
        new_node.parent = nearest_idx;

        if( !check_collision(tree[nearest_idx], new_node) ){

            new_node.cost = tree[nearest_idx].cost + line_cost(tree[nearest_idx], new_node);

            // connect along minimum cost path
            std::vector<int> Xnear = near(tree, new_node);
            for(const int& xnear : Xnear){

                double near2new_cost = tree[xnear].cost + line_cost(tree[xnear], new_node);
                
                if( !check_collision(tree[xnear], new_node ) && near2new_cost < new_node.cost ){
                    
                    new_node.parent = xnear;
                    new_node.cost = near2new_cost;

                }  
            }
            tree.push_back(new_node);

            // rewire
            for(const int& xnear : Xnear){

                double new2near_cost = new_node.cost + line_cost(tree[xnear], new_node);
                if( !check_collision(tree[xnear], new_node ) && new2near_cost < tree[xnear].cost ){
                    
                    tree[xnear].parent = tree.size() - 1;
                    tree[xnear].cost = new2near_cost;

                }  
            }

            if( is_goal(new_node, goal.x, goal.y) ){
                std::vector<RRT_Node> path = find_path(tree, new_node);
                publish_path(path);
                pure_pursuit(path);
                if( TREE ){
                    publish_tree(path, tree);
                }
                return;
            }
            else{
               
                if( TREE ){  
                    publish_tree(std::vector<RRT_Node>(), tree);
                }
            }
        }
    // path found as Path message
    }
}

void RRT::pure_pursuit(const std::vector<RRT_Node>& path ){

    for(int i = path.size() - 1; i >= 0 ; i--){

        // TODO: calculate curvature/steering angle
        float gamma = 2.f * path[i].y / pow(max_expansion_dist, 2.f);

        // TODO: publish drive message, don't forget to limit the steering angle.
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = 1.2;
        drive_msg.drive.steering_angle = max(-clip_val, min(gamma*P, clip_val));
        drive_pub_->publish(drive_msg);

    }

}


void RRT::publish_tree(const std::vector<RRT_Node>& path, const std::vector<RRT_Node>& tree ){


    std::vector<std::vector<RRT_Node>> branches;
    // std::vector<int> visited;
    // int id = 0;

    for( size_t i = tree.size() - 1; i > 0; i--){

        // if( std::find(visited.begin(), visited.end(), i) != visited.end()) continue;

        std::vector<RRT_Node> branch;
        RRT_Node node = tree[i];
        
        while( !node.is_root ){

            branch.push_back(node);
            node = tree[node.parent];

        }

        branch.push_back(tree[0]);
        branches.push_back(branch);

    }

    visualization_msgs::msg::MarkerArray delete_array = visualization_msgs::msg::MarkerArray();
    visualization_msgs::msg::Marker delete_marker = visualization_msgs::msg::Marker();
    delete_marker.header.frame_id = "ego_racecar/base_link";
    delete_marker.header.stamp = this->now();
    delete_marker.action = 3; // delete all
    delete_marker.ns = "tree";
    delete_array.markers.push_back(delete_marker);

    // for( size_t i = 0; i <= delete_size ; i++){
        // delete_marker.id = i;
    tree_pub_->publish(delete_array);
    // }

    visualization_msgs::msg::MarkerArray tree_marker = visualization_msgs::msg::MarkerArray();

    for(size_t i = 0 ; i < branches.size() ; i++){
        
        visualization_msgs::msg::Marker branch_marker = visualization_msgs::msg::Marker();
        branch_marker.header.frame_id = "ego_racecar/base_link";
        branch_marker.header.stamp = this->now();
        branch_marker.action = 0;
        branch_marker.ns = "tree";
        branch_marker.id = i;
        branch_marker.type = 4;

        branch_marker.scale.x = 0.01;
        branch_marker.color.a = 1.0;
        branch_marker.color.r = 1.0;
                
        for( size_t j = 0 ; j < branches[i].size() ; j++ ){
            
            geometry_msgs::msg::Point p = geometry_msgs::msg::Point();
            p.x = branches[i][j].x;
            p.y = branches[i][j].y;
            p.z = 0.05;

            branch_marker.points.push_back(p);

        }

        tree_marker.markers.push_back(branch_marker);

    }
    
    // add the chosen path
    if( !path.empty() ){
        visualization_msgs::msg::Marker path_marker = visualization_msgs::msg::Marker();
        path_marker.header.frame_id = "ego_racecar/base_link";
        path_marker.header.stamp = this->now();
        path_marker.action = 0; // delete all
        path_marker.ns = "tree";
        path_marker.id = branches.size();
        path_marker.type = 4;
        path_marker.scale.x = 0.03;
        path_marker.color.a = 1.0;
        path_marker.color.g = 1.0;

        for( size_t i = 0 ; i < path.size() ; i++ ){
                
                geometry_msgs::msg::Point p = geometry_msgs::msg::Point();
                p.x = path[i].x;
                p.y = path[i].y;
                p.z = 0.1;

                path_marker.points.push_back(p);

        }
        tree_marker.markers.push_back(path_marker);
    }
    tree_pub_->publish(tree_marker);

}


void RRT::publish_goal(const double& goal_x, const double& goal_y )
{
    
    visualization_msgs::msg::Marker goal_marker = visualization_msgs::msg::Marker();
    goal_marker.header.frame_id = "ego_racecar/base_link";
    goal_marker.header.stamp = this->now();
    goal_marker.action = 0;
    goal_marker.ns = "markers";
    goal_marker.type = 2;

    goal_marker.pose.position.x = goal_x;
    goal_marker.pose.position.y = goal_y;
    goal_marker.pose.position.z = 0.2;

    goal_marker.scale.x = 0.3;
    goal_marker.scale.y = 0.3;
    goal_marker.scale.z = 0.3;

    goal_marker.color.r = 1.0; 
    goal_marker.color.g = 0.0; 
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 1.0;

    goal_pub_->publish(goal_marker);

}

void RRT::publish_path(const std::vector<RRT_Node>& path ){

    nav_msgs::msg::Path path_msg = nav_msgs::msg::Path();
    path_msg.header.frame_id = "ego_racecar/base_link";

    for(auto it = path.rbegin(); it != path.rend(); it++ ){

        geometry_msgs::msg::PoseStamped pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.pose.position.x = it->x;
        pose_msg.pose.position.y = it->y;
        pose_msg.pose.position.z = 0.1;

        path_msg.poses.push_back(pose_msg);

    }

    path_pub_->publish(path_msg);

}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    // std::vector<double> sampled_point;
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    std::vector<double> sampled_point;

    // sample pixel centers
    sampled_point.emplace_back( x_dist(gen) );
    sampled_point.emplace_back( y_dist(gen) );
    
    return sampled_point;
}


int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

     // find the closest node

    std::pair<int, double> closest = {-1, std::numeric_limits<double>::max()};
    RRT_Node node;

    node.x = sampled_point[0];
    node.y = sampled_point[1];
    
    for(size_t n = 0; n < tree.size(); n++ ){

        double dist = std::sqrt((node.x-tree[n].x)*(node.x-tree[n].x) + (node.y-tree[n].y)*(node.y-tree[n].y));
        if( dist < closest.second ){
            closest.first = n;
            closest.second = dist;
        }
    }

    return closest.first;

}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    RRT_Node new_node;

    // Direction vector from nearest_node to sampled_point
    double dx = sampled_point[0] - nearest_node.x;
    double dy = sampled_point[1] - nearest_node.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // If the sampled point is within max_expansion_dist, use it directly
    if (dist < max_expansion_dist) {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    } else {
        // Normalize the direction vector and scale by max_expansion_dist
        dx /= dist;
        dy /= dist;
        new_node.x = nearest_node.x + dx * max_expansion_dist;
        new_node.y = nearest_node.y + dy * max_expansion_dist;
    }

    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    // find a and b in the pixel world
    std::vector<double> a = { new_node.x / map_resolution, new_node.y / map_resolution + map_wid_px};
    std::vector<double> b = { nearest_node.x / map_resolution, nearest_node.y / map_resolution + map_wid_px };

    // set up bresenhams 
    double dy = std::abs(a[1] - b[1]);
    double dx = std::abs(a[0] - b[0]);

    // find the major axis 
    int i = dx > dy ? 0 : 1;
    int j = dx > dy ? 1 : 0;

    if( a[i] > b[i] ){
        std::swap(a, b);
    }

    int t1 = (int) std::floor(a[i]);
    int t2 = (int) std::floor(b[i]);

    for( int u = t1; u < t2; u++){

        double w = ( u - a[i] ) / ( b[i] - a[i] );
        double v = w * ( b[j] - a[j] ) + a[j];

        double idx_x = std::floor(u); // px center -> px
        double idx_y = std::floor(v); // px center -> px

        if( dx < dy ){
            std::swap(idx_x, idx_y);
        }

        // check if occupied
        int idx = idx_x + idx_y * map_y;
        if( idx < 0 || idx >= total_boxes ) continue;

        if( occ_grid.data[idx] == 100 ){
            return true; 
        }

    } 

    return false;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal


    double dist = pow( pow(latest_added_node.x - goal_x, 2) + pow(latest_added_node.y + goal_y, 2), 0.5 );

    return dist < goal_threshold;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<RRT_Node> found_path;
    RRT_Node node = latest_added_node;

    while( !node.is_root ){
        found_path.push_back(node);
        node = tree[node.parent];
    }
    found_path.push_back(tree[0]);

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method
    do{
        cost += node.cost;
        node = tree[node.parent];
    }
    while(!node.is_root);

    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = std::sqrt((n1.x-n2.x)*(n1.x-n2.x) + (n1.y-n2.y)*(n1.y-n2.y));

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node>& tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    for (size_t i = 0; i < tree.size(); ++i) {
        double distance = std::sqrt(std::pow(tree[i].x - node.x, 2) + std::pow(tree[i].y - node.y, 2));
        if (distance < max_expansion_dist * 2.) {
            neighborhood.push_back(i);
        }
    }
    return neighborhood;
}

inline void parse_wps(const std::string& fname, vector<WayPoint>& wps, visualization_msgs::msg::MarkerArray& wps_viz){

        ifstream file(fname);

        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << fname << std::endl;
            return; // Exit the function if the file couldn't be opened
        }
        else{
            std::cout << "Opened: " << fname << std::endl;
        }

        // Read data
        string line;
        int id = 0;

        int counter = 0;

        while (getline(file, line)) {

            ++counter;
            if (counter % 10 != 0) continue;

            stringstream ss(line);
            string cell;

            WayPoint wp;

            // Read x coordinate
            getline(ss, cell, ',');
            wp.x = stod(cell); 

            // Read y coordinate
            getline(ss, cell, ',');
            wp.y = stod(cell); 

            // Add the waypoint to the vector
            wps.push_back(wp);

            // make waypoint marker
            visualization_msgs::msg::Marker wp_viz;
            wp_viz.header.frame_id = "map";
            wp_viz.type = visualization_msgs::msg::Marker::CYLINDER;
            wp_viz.id = id++;
            wp_viz.pose.position.x = wp.x;
            wp_viz.pose.position.y = wp.y;
            wp_viz.scale.x = 0.1; 
            wp_viz.scale.y = 0.1; 
            wp_viz.scale.z = 0.1;
            wp_viz.color.a = 1.0;
            wp_viz.color.r = 0.0;
            wp_viz.color.g = 1.0;
            wp_viz.color.b = 0.0;

            // save marker values
            wps_viz.markers.push_back(wp_viz);

        }

        std::cout << wps_viz.markers.size() << " waypoints added to track!" << std::endl;
        file.close();

    }