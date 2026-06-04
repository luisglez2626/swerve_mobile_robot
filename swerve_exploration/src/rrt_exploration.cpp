// File: ~/ros2_ws/src/swerve_robot/swerve_exploration/src/rrt_exploration.cpp

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <random>
#include <vector>
#include <cmath>
#include <chrono>
#include <utility>
#include <mutex>
#include <nanoflann.hpp>

using namespace std::chrono_literals;

// --- Data Structures for RRT and KD-Tree ---
struct RRTNode {
    double x;
    double y;
    int parent_idx;
};

// Adapter required by nanoflann to read our RRTNode structure
struct PointCloud {
    std::vector<RRTNode> pts;
    inline size_t kdtree_get_point_count() const { return pts.size(); }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return pts[idx].x;
        else return pts[idx].y;
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

// Dynamic KD-Tree type definition
typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud>,
    PointCloud, 2 /* dimensionality */
> KDTreeDynamic;

struct BlacklistPoint {
    double x;
    double y;
    rclcpp::Time timestamp;
};

struct FrontierCandidate {
    double x;
    double y;
    double distance;
};


class RRTExploration : public rclcpp::Node {
public:
    RRTExploration() : Node("rrt_exploration"), is_exploring_(false), exploration_finished_(false) {
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&RRTExploration::mapCallback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");
            
        tree_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_tree", 10);

        // Initialize KD-Tree
        kd_tree_ = std::make_unique<KDTreeDynamic>(2, cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10));

        timer_ = this->create_wall_timer(
            2000ms, std::bind(&RRTExploration::explorationLoop, this));
            
        RCLCPP_INFO(this->get_logger(), "RRT Exploration node started with dynamic KD-Tree enabled.");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_map_ = msg;
    }

    void explorationLoop() {
        nav_msgs::msg::OccupancyGrid::SharedPtr active_map;
        {
            // Safely grab a reference to the latest map without blocking callbacks
            std::lock_guard<std::mutex> lock(map_mutex_);
            active_map = current_map_;
        }

        if (!active_map || exploration_finished_) {
            return;
        }

        auto now_time = this->now();

        // Expire old blacklist entries (60 second decay)
        blacklist_.erase(std::remove_if(blacklist_.begin(), blacklist_.end(),
            [&](const BlacklistPoint& pt) {
                return (now_time - pt.timestamp).seconds() > 60.0;
            }), blacklist_.end());

        if (is_exploring_) {
            if ((now_time - goal_start_time_).seconds() > 40.0) {
                RCLCPP_WARN(this->get_logger(), "Watchdog triggered: Nav2 is stuck. Blacklisting and recalculating.");
                nav_client_->async_cancel_all_goals();
                blacklist_.push_back({current_goal_.pose.position.x, current_goal_.pose.position.y, now_time});
                is_exploring_ = false;
            }
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        double start_x = transform.transform.translation.x;
        double start_y = transform.transform.translation.y;

        geometry_msgs::msg::PoseStamped frontier_goal;
        if (findFrontierRRT(start_x, start_y, frontier_goal, active_map)) {
            sendNavGoal(frontier_goal);
        } else {
            exploration_finished_ = true;
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Exploration physically complete. Autonomous RRT timer deactivated.");
        }
    }

    bool findFrontierRRT(double start_x, double start_y, geometry_msgs::msg::PoseStamped& goal, const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {
        
        // Initialize the tree with the robot's current position if it's empty
        if (cloud_.pts.empty()) {
            cloud_.pts.push_back({start_x, start_y, -1});
            kd_tree_->addPoints(0, 0);
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        
        double map_width_m = map->info.width * map->info.resolution;
        double map_height_m = map->info.height * map->info.resolution;
        double origin_x = map->info.origin.position.x;
        double origin_y = map->info.origin.position.y;

        std::uniform_real_distribution<> x_dist(origin_x, origin_x + map_width_m);
        std::uniform_real_distribution<> y_dist(origin_y, origin_y + map_height_m);

        double step_size = 0.5;
        int max_iterations = 20000;
        
        std::vector<FrontierCandidate> candidates;

        for (int i = 0; i < max_iterations; ++i) {
            double rand_x = x_dist(gen);
            double rand_y = y_dist(gen);

            int nearest_idx = getNearestNode(rand_x, rand_y);
            RRTNode nearest = cloud_.pts[nearest_idx];

            double theta = std::atan2(rand_y - nearest.y, rand_x - nearest.x);
            double new_x = nearest.x + step_size * std::cos(theta);
            double new_y = nearest.y + step_size * std::sin(theta);

            int collision_state = checkLineCollision(nearest.x, nearest.y, new_x, new_y, map);
            
            if (collision_state == 0) { // Free Space
                cloud_.pts.push_back({new_x, new_y, nearest_idx});
                kd_tree_->addPoints(cloud_.pts.size() - 1, cloud_.pts.size() - 1);
            } else if (collision_state == -1) { // Hit Unknown Space (Frontier)
                if (isFrontierSafe(new_x, new_y, map)) {
                    double dist = std::hypot(new_x - start_x, new_y - start_y);
                    candidates.push_back({new_x, new_y, dist});
                    
                    // Stop searching once we have a solid batch of candidates
                    if (candidates.size() >= 5) break; 
                }
            }
        }

        publishTreeMarker();

        // Sort candidates and pick the optimal (closest) one
        if (!candidates.empty()) {
            double best_dist = std::numeric_limits<double>::max();
            FrontierCandidate best_cand;
            for (const auto& c : candidates) {
                if (c.distance < best_dist) {
                    best_dist = c.distance;
                    best_cand = c;
                }
            }
            
            goal.header.frame_id = "map";
            goal.header.stamp = this->now();
            goal.pose.position.x = best_cand.x;
            goal.pose.position.y = best_cand.y;
            goal.pose.orientation.w = 1.0;
            return true;
        }

        return false;
    }

    int getNearestNode(double x, double y) {
        double query_pt[2] = {x, y};
        size_t ret_index;
        double out_dist_sqr;
        nanoflann::KNNResultSet<double> resultSet(1);
        resultSet.init(&ret_index, &out_dist_sqr);
        kd_tree_->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
        return ret_index;
    }

    // Uses Bresenham's line algorithm to perfectly step through grid cells
    int checkLineCollision(double x0, double y0, double x1, double y1, const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {
        double res = map->info.resolution;
        double origin_x = map->info.origin.position.x;
        double origin_y = map->info.origin.position.y;

        int x0_grid = std::floor((x0 - origin_x) / res);
        int y0_grid = std::floor((y0 - origin_y) / res);
        int x1_grid = std::floor((x1 - origin_x) / res);
        int y1_grid = std::floor((y1 - origin_y) / res);

        int dx = std::abs(x1_grid - x0_grid);
        int dy = std::abs(y1_grid - y0_grid);
        int sx = x0_grid < x1_grid ? 1 : -1;
        int sy = y0_grid < y1_grid ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (x0_grid < 0 || x0_grid >= (int)map->info.width || y0_grid < 0 || y0_grid >= (int)map->info.height) {
                return 100;
            }

            int idx = y0_grid * map->info.width + x0_grid;
            int val = map->data[idx];
            
            if (val == -1) return -1; // Frontier
            if (val > 50) return 1;   // Obstacle

            if (x0_grid == x1_grid && y0_grid == y1_grid) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0_grid += sx; }
            if (e2 < dx) { err += dx; y0_grid += sy; }
        }
        return 0; // Free space
    }

    bool isFrontierSafe(double x, double y, const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {
        for (const auto& bad_pt : blacklist_) {
            if (std::hypot(bad_pt.x - x, bad_pt.y - y) < 0.5) {
                return false; 
            }
        }

        double safe_radius = 0.65; 
        double res = map->info.resolution;
        int cells = std::ceil(safe_radius / res);
        double origin_x = map->info.origin.position.x;
        double origin_y = map->info.origin.position.y;

        for (int dx = -cells; dx <= cells; ++dx) {
            for (int dy = -cells; dy <= cells; ++dy) {
                if (dx * dx + dy * dy <= cells * cells) {
                    double cx = x + dx * res;
                    double cy = y + dy * res;
                    
                    int grid_x = std::floor((cx - origin_x) / res);
                    int grid_y = std::floor((cy - origin_y) / res);

                    if (grid_x < 0 || grid_x >= (int)map->info.width || grid_y < 0 || grid_y >= (int)map->info.height) {
                        return false; 
                    }

                    int index = grid_y * map->info.width + grid_x;
                    int val = map->data[index];
                    
                    if (val > 50 && val <= 100) {
                        return false; 
                    }
                }
            }
        }
        return true;
    }

    void sendNavGoal(const geometry_msgs::msg::PoseStamped& goal) {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available.");
            return;
        }

        is_exploring_ = true;
        current_goal_ = goal; 
        goal_start_time_ = this->now();

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = 
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_WARN(this->get_logger(), "Nav2 instantly REJECTED the goal. Blacklisting.");
                    blacklist_.push_back({current_goal_.pose.position.x, current_goal_.pose.position.y, this->now()});
                    is_exploring_ = false;
                }
            };

        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
                is_exploring_ = false;
                if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_WARN(this->get_logger(), "Nav2 aborted. The area is inaccessible. Adding to blacklist.");
                    blacklist_.push_back({current_goal_.pose.position.x, current_goal_.pose.position.y, this->now()});
                }
            };

        nav_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Heading to optimal frontier at x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
    }

    void publishTreeMarker() {
        visualization_msgs::msg::Marker tree_marker;
        tree_marker.header.frame_id = "map";
        tree_marker.header.stamp = this->now();
        tree_marker.ns = "rrt_exploration";
        tree_marker.id = 0;
        tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        tree_marker.action = visualization_msgs::msg::Marker::ADD;
        tree_marker.pose.orientation.w = 1.0;
        tree_marker.scale.x = 0.02; 
        tree_marker.color.r = 1.0;
        tree_marker.color.g = 0.0;
        tree_marker.color.b = 0.0;
        tree_marker.color.a = 0.8;

        for (size_t i = 1; i < cloud_.pts.size(); ++i) {
            geometry_msgs::msg::Point p1, p2;
            p1.x = cloud_.pts[i].x;
            p1.y = cloud_.pts[i].y;
            p1.z = 0.1;
            
            p2.x = cloud_.pts[cloud_.pts[i].parent_idx].x;
            p2.y = cloud_.pts[cloud_.pts[i].parent_idx].y;
            p2.z = 0.1;

            tree_marker.points.push_back(p1);
            tree_marker.points.push_back(p2);
        }
        tree_pub_->publish(tree_marker);
    }

    // ROS 2 Comms
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State Data
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    std::mutex map_mutex_;
    bool is_exploring_;
    bool exploration_finished_;
    geometry_msgs::msg::PoseStamped current_goal_;
    std::vector<BlacklistPoint> blacklist_; 
    rclcpp::Time goal_start_time_;

    // KD-Tree Data
    PointCloud cloud_;
    std::unique_ptr<KDTreeDynamic> kd_tree_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTExploration>());
    rclcpp::shutdown();
    return 0;
}