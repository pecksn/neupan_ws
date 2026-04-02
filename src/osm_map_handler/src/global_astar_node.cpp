#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <algorithm>

struct Node {
    int x, y;
    float g, h, f;
    Node* parent;

    Node(int x, int y, float g, float h, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}
};

struct CompareNode {
    bool operator()(Node* n1, Node* n2) {
        return n1->f > n2->f;
    }
};

class GlobalAStarPlanner {
public:
    GlobalAStarPlanner() {
        ros::NodeHandle nh("~");
        nh.param("lookahead_dist", lookahead_dist_, 15.0);

        map_sub_ = n_.subscribe("/map", 1, &GlobalAStarPlanner::mapCallback, this);
        goal_sub_ = n_.subscribe("/global_goal", 1, &GlobalAStarPlanner::goalCallback, this);
        odom_sub_ = n_.subscribe("/odometry/global", 1, &GlobalAStarPlanner::odomCallback, this);

        path_pub_ = n_.advertise<nav_msgs::Path>("/global_astar_path", 1);
        hybrid_goal_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/hybrid_astar_subgoal", 1);

        timer_ = n_.createTimer(ros::Duration(1.0), &GlobalAStarPlanner::updateLocalGoal, this);
        
        ROS_INFO("C++ Global A* Planner Initialized for Outdoor OSM.");
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber map_sub_, goal_sub_, odom_sub_;
    ros::Publisher path_pub_, hybrid_goal_pub_;
    ros::Timer timer_;

    nav_msgs::OccupancyGrid::ConstPtr map_ptr_;
    geometry_msgs::Pose robot_pose_;
    std::vector<geometry_msgs::Point> path_points_;
    double lookahead_dist_;
    bool has_odom_ = false;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_ptr_ = msg;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_pose_ = msg->pose.pose;
        has_odom_ = true;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!map_ptr_ || !has_odom_) {
            ROS_WARN("Global A*: Map or Odom not received yet!");
            return;
        }

        double res = map_ptr_->info.resolution;
        double ox = map_ptr_->info.origin.position.x;
        double oy = map_ptr_->info.origin.position.y;

        int sx = (robot_pose_.position.x - ox) / res;
        int sy = (robot_pose_.position.y - oy) / res;
        int gx = (msg->pose.position.x - ox) / res;
        int gy = (msg->pose.position.y - oy) / res;

        int w = map_ptr_->info.width;
        int h = map_ptr_->info.height;

        if (sx < 0 || sx >= w || sy < 0 || sy >= h || gx < 0 || gx >= w || gy < 0 || gy >= h) {
            ROS_WARN("Global A*: Start or Goal out of bounds!");
            return;
        }

        ROS_INFO("Computing C++ A* path...");
        std::vector<std::pair<int, int>> grid_path = runAStar(sx, sy, gx, gy);
        
        if (grid_path.empty()) {
            ROS_WARN("Global A* Failed!");
            return;
        }

        path_points_.clear();
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();

        for (auto& p : grid_path) {
            geometry_msgs::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = p.first * res + ox + res/2.0;
            ps.pose.position.y = p.second * res + oy + res/2.0;
            ps.pose.orientation.w = 1.0;
            path_msg.poses.push_back(ps);
            path_points_.push_back(ps.pose.position);
        }
        path_pub_.publish(path_msg);
        ROS_INFO("Global A* Path found with %zu points.", path_points_.size());
    }

    std::vector<std::pair<int, int>> runAStar(int sx, int sy, int gx, int gy) {
        int w = map_ptr_->info.width;
        int h = map_ptr_->info.height;
        auto to_idx = [&](int x, int y) { return (size_t)y * w + x; };

        std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
        std::unordered_map<size_t, Node*> all_nodes;

        Node* start = new Node(sx, sy, 0, hypot(gx-sx, gy-sy));
        open_list.push(start);
        all_nodes[to_idx(sx, sy)] = start;

        while (!open_list.empty()) {
            Node* current = open_list.top();
            open_list.pop();

            if (current->x == gx && current->y == gy) {
                std::vector<std::pair<int, int>> res;
                Node* temp = current;
                while (temp) {
                    res.push_back({temp->x, temp->y});
                    temp = temp->parent;
                }
                std::reverse(res.begin(), res.end());
                for (auto& pair : all_nodes) delete pair.second;
                return res;
            }

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = current->x + dx;
                    int ny = current->y + dy;

                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                    
                    size_t idx = to_idx(nx, ny);
                    int cost = map_ptr_->data[idx];
                    if (cost > 50 || cost == -1) continue;

                    float step = (dx != 0 && dy != 0) ? 1.414 : 1.0;
                    float new_g = current->g + step + (cost / 100.0);

                    if (all_nodes.find(idx) == all_nodes.end() || new_g < all_nodes[idx]->g) {
                        float new_h = hypot(gx-nx, gy-ny);
                        if (all_nodes.find(idx) == all_nodes.end()) {
                            Node* next = new Node(nx, ny, new_g, new_h, current);
                            all_nodes[idx] = next;
                            open_list.push(next);
                        } else {
                            all_nodes[idx]->g = new_g;
                            all_nodes[idx]->f = new_g + new_h;
                            all_nodes[idx]->parent = current;
                        }
                    }
                }
            }
        }
        for (auto& pair : all_nodes) delete pair.second;
        return {};
    }

    void updateLocalGoal(const ros::TimerEvent&) {
        if (path_points_.empty() || !has_odom_) return;

        double rx = robot_pose_.position.x;
        double ry = robot_pose_.position.y;
        
        size_t target_idx = path_points_.size() - 1;
        for (size_t i = 0; i < path_points_.size(); ++i) {
            if (hypot(path_points_[i].x - rx, path_points_[i].y - ry) > lookahead_dist_) {
                target_idx = i;
                break;
            }
        }

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position = path_points_[target_idx];
        
        double yaw = 0;
        if (target_idx < path_points_.size() - 1) {
            yaw = atan2(path_points_[target_idx+1].y - path_points_[target_idx].y, 
                        path_points_[target_idx+1].x - path_points_[target_idx].x);
        }
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        
        hybrid_goal_pub_.publish(goal);
        
        if (hypot(path_points_.back().x - rx, path_points_.back().y - ry) < 1.0) {
            ROS_INFO_THROTTLE(10, "Global Goal Reached!");
            path_points_.clear();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_astar_planner");
    GlobalAStarPlanner gap;
    ros::spin();
    return 0;
}
