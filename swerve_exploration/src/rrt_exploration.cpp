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

using namespace std::chrono_literals;

struct RRTNode {
    double x;
    double y;
    int parent_idx;
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

        timer_ = this->create_wall_timer(
            2000ms, std::bind(&RRTExploration::explorationLoop, this));
            
        RCLCPP_INFO(this->get_logger(), "RRT Exploration node started.");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = msg;
    }

    void explorationLoop() {
        if (!current_map_ || exploration_finished_) {
            return;
        }

        if (is_exploring_) {
            if ((this->now() - goal_start_time_).seconds() > 40.0) {
                RCLCPP_WARN(this->get_logger(), "Watchdog triggered: Nav2 is stuck. Force canceling and blacklisting.");
                nav_client_->async_cancel_all_goals();
                blacklist_.push_back({current_goal_.pose.position.x, current_goal_.pose.position.y});
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
        if (findFrontierRRT(start_x, start_y, frontier_goal)) {
            sendNavGoal(frontier_goal);
        } else {
            // Lock the node. Once no frontiers are found, hand control permanently to the operator.
            exploration_finished_ = true;
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Exploration physically complete. Autonomous RRT timer deactivated. You may now safely send GUI waypoints.");
        }
    }

    bool findFrontierRRT(double start_x, double start_y, geometry_msgs::msg::PoseStamped& goal) {
        std::vector<RRTNode> tree;
        tree.push_back({start_x, start_y, -1});

        std::random_device rd;
        std::mt19937 gen(rd());
        
        double map_width_m = current_map_->info.width * current_map_->info.resolution;
        double map_height_m = current_map_->info.height * current_map_->info.resolution;
        double origin_x = current_map_->info.origin.position.x;
        double origin_y = current_map_->info.origin.position.y;

        std::uniform_real_distribution<> x_dist(origin_x, origin_x + map_width_m);
        std::uniform_real_distribution<> y_dist(origin_y, origin_y + map_height_m);

        double step_size = 0.5;
        int max_iterations = 4000;
        bool found_frontier = false;

        for (int i = 0; i < max_iterations; ++i) {
            double rand_x = x_dist(gen);
            double rand_y = y_dist(gen);

            int nearest_idx = getNearestNode(tree, rand_x, rand_y);
            RRTNode nearest = tree[nearest_idx];

            double theta = std::atan2(rand_y - nearest.y, rand_x - nearest.x);
            double new_x = nearest.x + step_size * std::cos(theta);
            double new_y = nearest.y + step_size * std::sin(theta);

            int collision_state = checkLineCollision(nearest.x, nearest.y, new_x, new_y);
            
            if (collision_state == 0) {
                tree.push_back({new_x, new_y, nearest_idx});
            } else if (collision_state == -1) {
                if (isFrontierSafe(new_x, new_y)) {
                    goal.header.frame_id = "map";
                    goal.header.stamp = this->now();
                    goal.pose.position.x = new_x;
                    goal.pose.position.y = new_y;
                    goal.pose.orientation.w = 1.0;
                    found_frontier = true;
                    break;
                }
            }
        }

        publishTreeMarker(tree);
        return found_frontier;
    }

    void publishTreeMarker(const std::vector<RRTNode>& tree) {
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

        for (size_t i = 1; i < tree.size(); ++i) {
            geometry_msgs::msg::Point p1, p2;
            p1.x = tree[i].x;
            p1.y = tree[i].y;
            p1.z = 0.1;
            
            p2.x = tree[tree[i].parent_idx].x;
            p2.y = tree[tree[i].parent_idx].y;
            p2.z = 0.1;

            tree_marker.points.push_back(p1);
            tree_marker.points.push_back(p2);
        }
        tree_pub_->publish(tree_marker);
    }

    bool isFrontierSafe(double x, double y) {
        for (const auto& bad_pt : blacklist_) {
            if (std::hypot(bad_pt.first - x, bad_pt.second - y) < 1.5) {
                return false; 
            }
        }

        // 0.62m physical constraint + 0.03m grid snap tolerance
        double safe_radius = 0.65; 
        double res = current_map_->info.resolution;
        int cells = std::ceil(safe_radius / res);

        for (int dx = -cells; dx <= cells; ++dx) {
            for (int dy = -cells; dy <= cells; ++dy) {
                if (dx * dx + dy * dy <= cells * cells) {
                    double cx = x + dx * res;
                    double cy = y + dy * res;
                    int val = getMapValue(cx, cy);
                    
                    if (val > 50 && val <= 100) {
                        return false; 
                    }
                }
            }
        }
        return true;
    }

    int getNearestNode(const std::vector<RRTNode>& tree, double x, double y) {
        int nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < tree.size(); ++i) {
            double dist = std::hypot(tree[i].x - x, tree[i].y - y);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        return nearest_idx;
    }

    int checkLineCollision(double x0, double y0, double x1, double y1) {
        int steps = 10;
        double dx = (x1 - x0) / steps;
        double dy = (y1 - y0) / steps;

        for (int i = 0; i <= steps; ++i) {
            double cx = x0 + i * dx;
            double cy = y0 + i * dy;
            
            int map_val = getMapValue(cx, cy);
            if (map_val == -1) {
                return -1; 
            }
            if (map_val > 50) {
                return 1; 
            }
        }
        return 0; 
    }

    int getMapValue(double x, double y) {
        double origin_x = current_map_->info.origin.position.x;
        double origin_y = current_map_->info.origin.position.y;
        double res = current_map_->info.resolution;

        int grid_x = std::floor((x - origin_x) / res);
        int grid_y = std::floor((y - origin_y) / res);

        if (grid_x < 0 || grid_x >= (int)current_map_->info.width ||
            grid_y < 0 || grid_y >= (int)current_map_->info.height) {
            return 100;
        }

        int index = grid_y * current_map_->info.width + grid_x;
        return current_map_->data[index];
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
                    blacklist_.push_back({current_goal_.pose.position.x, current_goal_.pose.position.y});
                    is_exploring_ = false;
                }
            };

        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
                is_exploring_ = false;
               
                if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_WARN(this->get_logger(), "Nav2 aborted. The area is inaccessible. Adding to blacklist.");
                    blacklist_.push_back({current_goal_.pose.position.x, current_goal_.pose.position.y});
                }
            };

        nav_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Heading to new frontier at x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    bool is_exploring_;
    bool exploration_finished_;
    geometry_msgs::msg::PoseStamped current_goal_;
    std::vector<std::pair<double, double>> blacklist_; 
    rclcpp::Time goal_start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTExploration>());
    rclcpp::shutdown();
    return 0;
}