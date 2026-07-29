#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>

namespace pt = boost::property_tree;

struct XYPoint {
    double x{0.0};
    double y{0.0};
};

struct EdgeInfo {
    std::string start_id;
    std::string end_id;
    double length{0.0};
    double road_width{0.0};
    int lanes{0};
    bool forward_allowed{true};
    bool backward_allowed{true};
};

struct SnapInfo {
    XYPoint point;
    double dist{std::numeric_limits<double>::infinity()};
    std::string start_id;
    std::string end_id;
    double t{0.0};
    double edge_len{0.0};
    bool forward_allowed{true};
    bool backward_allowed{true};
    bool valid{false};
};

class OSMGraphAStarPlanner {
public:
    OSMGraphAStarPlanner() : private_nh_("~") {
        private_nh_.param("osm_file", osm_file_, std::string("/home/nuc/lxw/ros_maps/map.osm"));
        private_nh_.param("default_lat", origin_lat_, 23.0398);
        private_nh_.param("default_lon", origin_lon_, 113.39002);
        private_nh_.param("lookahead_dist", lookahead_dist_, 4.0);
        private_nh_.param("strict_road_mode", strict_road_mode_, true);
        private_nh_.param("goal_snap_max_dist", goal_snap_max_dist_, 50.0);
        private_nh_.param("start_snap_max_dist", start_snap_max_dist_, 50.0);
        private_nh_.param("path_straight_spacing", path_straight_spacing_, 2.5);
        private_nh_.param("path_turn_spacing", path_turn_spacing_, 0.8);
        private_nh_.param("path_turn_angle_deg_low", path_turn_angle_deg_low_, 8.0);
        private_nh_.param("path_turn_angle_deg_high", path_turn_angle_deg_high_, 35.0);
        private_nh_.param("path_turn_dense_before", path_turn_dense_before_, 3.0);
        private_nh_.param("path_turn_dense_after", path_turn_dense_after_, 2.0);
        private_nh_.param("path_turn_recover_after", path_turn_recover_after_, 6.0);
        private_nh_.param("path_min_points", path_min_points_, 3);
        private_nh_.param("auto_set_initialpose", auto_set_initialpose_, true);
        private_nh_.param("initialpose_cov_xy", initialpose_cov_xy_, 2.0);
        private_nh_.param("initialpose_cov_yaw", initialpose_cov_yaw_, 0.25);
        private_nh_.param("publish_pose_markers", publish_pose_markers_, true);
        private_nh_.param("publish_snap_debug_markers", publish_snap_debug_markers_, true);
        private_nh_.param("gps_marker_timeout", gps_marker_timeout_, 2.0);
        private_nh_.param("enable_right_lane_offset", enable_right_lane_offset_, false);
        private_nh_.param("respect_oneway", respect_oneway_, true);
        private_nh_.param("default_lane_width", default_lane_width_, 3.0);
        private_nh_.param("default_road_width", default_road_width_, 6.0);
        private_nh_.param("right_lane_offset_ratio", right_lane_offset_ratio_, 0.25);
        private_nh_.param("max_right_lane_offset", max_right_lane_offset_, 2.0);
        private_nh_.param("vector_map_line_width", vector_map_line_width_, 0.25);
        private_nh_.param("vector_map_z", vector_map_z_, 0.35);
        private_nh_.param("vector_map_alpha", vector_map_alpha_, 1.0);
        private_nh_.param("path_point_marker_scale", path_point_marker_scale_, 0.85);
        private_nh_.param("path_line_marker_width", path_line_marker_width_, 0.10);
        private_nh_.param("path_endpoint_marker_scale", path_endpoint_marker_scale_, 1.35);
        private_nh_.param("snap_marker_scale", snap_marker_scale_, 1.10);
        private_nh_.param("snap_link_marker_width", snap_link_marker_width_, 0.08);

        gps_marker_timeout_ = std::max(0.2, gps_marker_timeout_);
        path_straight_spacing_ = std::max(0.3, path_straight_spacing_);
        path_turn_spacing_ = std::max(0.2, std::min(path_turn_spacing_, path_straight_spacing_));
        vector_map_line_width_ = std::max(0.05, vector_map_line_width_);
        vector_map_alpha_ = std::max(0.0, std::min(1.0, vector_map_alpha_));
        path_turn_angle_low_rad_ = degToRad(path_turn_angle_deg_low_);
        path_turn_angle_high_rad_ = degToRad(std::max(path_turn_angle_deg_low_, path_turn_angle_deg_high_));
        path_min_points_ = std::max(2, path_min_points_);
        default_lane_width_ = std::max(1.5, default_lane_width_);
        default_road_width_ = std::max(default_lane_width_, default_road_width_);
        right_lane_offset_ratio_ = std::max(0.0, std::min(0.5, right_lane_offset_ratio_));
        max_right_lane_offset_ = std::max(0.0, max_right_lane_offset_);
        path_point_marker_scale_ = std::max(0.2, path_point_marker_scale_);
        path_line_marker_width_ = std::max(0.02, path_line_marker_width_);
        path_endpoint_marker_scale_ = std::max(0.3, path_endpoint_marker_scale_);
        snap_marker_scale_ = std::max(0.2, snap_marker_scale_);
        snap_link_marker_width_ = std::max(0.02, snap_link_marker_width_);

        wheelable_ways_ = {
            "primary", "secondary", "tertiary", "unclassified",
            "residential", "service", "living_street", "trunk"
        };

        path_pub_ = nh_.advertise<nav_msgs::Path>("/osm_global_path", 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/osm_vector_map", 1, true);
        subgoal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/hybrid_astar_subgoal", 1);
        initialpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

        map_sub_ = nh_.subscribe("/map", 1, &OSMGraphAStarPlanner::mapCallback, this);
        odom_sub_ = nh_.subscribe("/odometry/global", 1, &OSMGraphAStarPlanner::odomCallback, this);
        gps_sub_ = nh_.subscribe("/gps/fix", 1, &OSMGraphAStarPlanner::gpsCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &OSMGraphAStarPlanner::goalCallback, this);
        internal_goal_sub_ = nh_.subscribe("/teb_test/osm_goal", 1, &OSMGraphAStarPlanner::goalCallback, this);
        clicked_point_sub_ = nh_.subscribe("/clicked_point", 1, &OSMGraphAStarPlanner::clickedPointCallback, this);

        loadGraph();
        ROS_INFO(
            "OSM C++ planner config: strict_road_mode=%s, start_snap_max_dist=%.2fm, goal_snap_max_dist=%.2fm, path_spacing=%.2f/%.2fm, right_lane_offset=%s (ratio=%.2f max=%.2fm), respect_oneway=%s",
            strict_road_mode_ ? "true" : "false",
            start_snap_max_dist_,
            goal_snap_max_dist_,
            path_straight_spacing_,
            path_turn_spacing_,
            enable_right_lane_offset_ ? "true" : "false",
            right_lane_offset_ratio_,
            max_right_lane_offset_,
            respect_oneway_ ? "true" : "false");

        timer_ = nh_.createTimer(ros::Duration(0.1), &OSMGraphAStarPlanner::timerCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher path_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher subgoal_pub_;
    ros::Publisher initialpose_pub_;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber internal_goal_sub_;
    ros::Subscriber clicked_point_sub_;
    ros::Timer timer_;

    std::string osm_file_;
    double origin_lat_{23.0398};
    double origin_lon_{113.39002};
    const double earth_radius_{6378137.0};

    double lookahead_dist_{4.0};
    bool strict_road_mode_{true};
    double goal_snap_max_dist_{50.0};
    double start_snap_max_dist_{50.0};

    double path_straight_spacing_{2.5};
    double path_turn_spacing_{0.8};
    double path_turn_angle_deg_low_{8.0};
    double path_turn_angle_deg_high_{35.0};
    double path_turn_angle_low_rad_{0.0};
    double path_turn_angle_high_rad_{0.0};
    double path_turn_dense_before_{3.0};
    double path_turn_dense_after_{2.0};
    double path_turn_recover_after_{6.0};
    int path_min_points_{3};

    bool auto_set_initialpose_{true};
    double initialpose_cov_xy_{2.0};
    double initialpose_cov_yaw_{0.25};
    bool publish_pose_markers_{true};
    bool publish_snap_debug_markers_{true};
    double gps_marker_timeout_{2.0};
    bool enable_right_lane_offset_{false};
    bool respect_oneway_{true};
    double default_lane_width_{3.0};
    double default_road_width_{6.0};
    double right_lane_offset_ratio_{0.25};
    double max_right_lane_offset_{2.0};
    double vector_map_line_width_{0.25};
    double vector_map_z_{0.35};
    double vector_map_alpha_{1.0};
    double path_point_marker_scale_{0.85};
    double path_line_marker_width_{0.10};
    double path_endpoint_marker_scale_{1.35};
    double snap_marker_scale_{1.10};
    double snap_link_marker_width_{0.08};

    std::set<std::string> wheelable_ways_;
    std::unordered_map<std::string, XYPoint> nodes_;
    std::unordered_map<std::string, std::vector<std::pair<std::string, double>>> adjacency_;
    std::vector<EdgeInfo> edges_;
    std::vector<XYPoint> current_path_;

    double robot_x_{0.0};
    double robot_y_{0.0};
    double robot_yaw_{0.0};
    bool has_odom_{false};
    bool has_map_{false};
    bool has_gps_{false};
    double gps_x_{0.0};
    double gps_y_{0.0};
    ros::Time last_gps_recv_;
    bool initialpose_sent_{false};

    static double hypot2d(double x1, double y1, double x2, double y2) {
        return std::hypot(x1 - x2, y1 - y2);
    }

    static double degToRad(double deg) {
        return deg * M_PI / 180.0;
    }

    static geometry_msgs::Point toRosPoint(const XYPoint& point, double z = 0.0) {
        geometry_msgs::Point ros_point;
        ros_point.x = point.x;
        ros_point.y = point.y;
        ros_point.z = z;
        return ros_point;
    }

    double yawFromQuaternion(const geometry_msgs::Quaternion& q) const {
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    geometry_msgs::Quaternion quaternionFromYaw(double yaw) const {
        geometry_msgs::Quaternion q;
        const double half = 0.5 * yaw;
        q.z = std::sin(half);
        q.w = std::cos(half);
        return q;
    }

    XYPoint latLonToXY(double lat, double lon) const {
        XYPoint point;
        point.x = (lon - origin_lon_) * (M_PI / 180.0) * earth_radius_ * std::cos(origin_lat_ * M_PI / 180.0);
        point.y = (lat - origin_lat_) * (M_PI / 180.0) * earth_radius_;
        return point;
    }

    static double parsePositiveNumber(const std::string& value, double fallback = 0.0) {
        if (value.empty()) {
            return fallback;
        }
        char* end = nullptr;
        const double parsed = std::strtod(value.c_str(), &end);
        return (end != value.c_str() && std::isfinite(parsed) && parsed > 0.0) ? parsed : fallback;
    }

    static int parsePositiveInt(const std::string& value, int fallback = 0) {
        const double parsed = parsePositiveNumber(value, static_cast<double>(fallback));
        return parsed > 0.0 ? std::max(1, static_cast<int>(std::lround(parsed))) : fallback;
    }

    double fallbackRoadWidth(const std::string& highway) const {
        if (highway == "trunk" || highway == "primary") {
            return 10.5;
        }
        if (highway == "secondary") {
            return 9.0;
        }
        if (highway == "tertiary") {
            return 7.5;
        }
        if (highway == "living_street") {
            return 5.0;
        }
        return default_road_width_;
    }

    void loadGraph() {
        ROS_INFO("Filtering OSM for drivable roads: %s", osm_file_.c_str());

        pt::ptree tree;
        try {
            pt::read_xml(osm_file_, tree);
        } catch (const std::exception& exc) {
            ROS_ERROR("Failed to load OSM file %s: %s", osm_file_.c_str(), exc.what());
            return;
        }

        std::unordered_map<std::string, XYPoint> temp_nodes;
        const auto& root = tree.get_child("osm");
        for (const auto& child : root) {
            if (child.first != "node") {
                continue;
            }
            const auto& attrs = child.second.get_child("<xmlattr>");
            const std::string id = attrs.get<std::string>("id", "");
            const double lat = attrs.get<double>("lat", 0.0);
            const double lon = attrs.get<double>("lon", 0.0);
            temp_nodes[id] = latLonToXY(lat, lon);
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "osm_roads";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = vector_map_line_width_;
        marker.color.r = 0.0;
        marker.color.g = 0.15;
        marker.color.b = 1.0;
        marker.color.a = vector_map_alpha_;
        marker.pose.orientation.w = 1.0;

        int way_count = 0;
        for (const auto& child : root) {
            if (child.first != "way") {
                continue;
            }

            std::unordered_map<std::string, std::string> tags;
            for (const auto& way_child : child.second) {
                if (way_child.first != "tag") {
                    continue;
                }
                const auto& attrs = way_child.second.get_child("<xmlattr>");
                tags[attrs.get<std::string>("k", "")] = attrs.get<std::string>("v", "");
            }

            const std::string hw_type = tags["highway"];

            if (wheelable_ways_.count(hw_type) == 0) {
                continue;
            }

            const int lanes = parsePositiveInt(tags["lanes"]);
            double road_width = parsePositiveNumber(tags["width"]);
            if (road_width <= 0.0 && lanes > 0) {
                road_width = lanes * default_lane_width_;
            }
            if (road_width <= 0.0) {
                road_width = fallbackRoadWidth(hw_type);
            }

            bool forward_allowed = true;
            bool backward_allowed = true;
            if (respect_oneway_) {
                const std::string oneway = tags["oneway"];
                const bool roundabout = tags["junction"] == "roundabout";
                if (oneway == "-1") {
                    forward_allowed = false;
                } else if (oneway == "yes" || oneway == "true" || oneway == "1" || roundabout) {
                    backward_allowed = false;
                }
            }

            std::vector<std::string> refs;
            for (const auto& way_child : child.second) {
                if (way_child.first == "nd") {
                    refs.push_back(way_child.second.get_child("<xmlattr>").get<std::string>("ref", ""));
                }
            }

            for (size_t i = 0; i < refs.size(); ++i) {
                const std::string& n1 = refs[i];
                if (temp_nodes.find(n1) == temp_nodes.end()) {
                    continue;
                }
                if (nodes_.find(n1) == nodes_.end()) {
                    nodes_[n1] = temp_nodes[n1];
                    adjacency_[n1] = {};
                }

                if (i + 1 >= refs.size()) {
                    continue;
                }

                const std::string& n2 = refs[i + 1];
                if (temp_nodes.find(n2) == temp_nodes.end()) {
                    continue;
                }
                if (nodes_.find(n2) == nodes_.end()) {
                    nodes_[n2] = temp_nodes[n2];
                    adjacency_[n2] = {};
                }

                const double dist = hypot2d(temp_nodes[n1].x, temp_nodes[n1].y, temp_nodes[n2].x, temp_nodes[n2].y);
                if (forward_allowed) {
                    adjacency_[n1].push_back({n2, dist});
                }
                if (backward_allowed) {
                    adjacency_[n2].push_back({n1, dist});
                }
                edges_.push_back({n1, n2, dist, road_width, lanes, forward_allowed, backward_allowed});
                marker.points.push_back(toRosPoint(temp_nodes[n1], vector_map_z_));
                marker.points.push_back(toRosPoint(temp_nodes[n2], vector_map_z_));
            }
            ++way_count;
        }

        marker_pub_.publish(marker);
        ROS_INFO("Drivable Graph loaded: %d ways, %zu nodes, %zu edges.", way_count, nodes_.size(), edges_.size());
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        has_map_ = (msg->info.width > 0 && msg->info.height > 0);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
        has_odom_ = true;
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) {
            return;
        }
        if (std::abs(msg->latitude) > 90.0 || std::abs(msg->longitude) > 180.0) {
            return;
        }
        const XYPoint xy = latLonToXY(msg->latitude, msg->longitude);
        gps_x_ = xy.x;
        gps_y_ = xy.y;
        last_gps_recv_ = ros::Time::now();
        has_gps_ = true;
    }

    std::pair<std::string, double> findNearestNode(double x, double y) const {
        double min_dist = std::numeric_limits<double>::infinity();
        std::string closest;
        for (const auto& node : nodes_) {
            const double dist = hypot2d(node.second.x, node.second.y, x, y);
            if (dist < min_dist) {
                min_dist = dist;
                closest = node.first;
            }
        }
        return {closest, min_dist};
    }

    XYPoint projectPointToSegment(double x, double y, const XYPoint& a, const XYPoint& b, double& t) const {
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double seg_len_sq = dx * dx + dy * dy;
        if (seg_len_sq <= 1e-9) {
            t = 0.0;
            return a;
        }
        t = ((x - a.x) * dx + (y - a.y) * dy) / seg_len_sq;
        t = std::max(0.0, std::min(1.0, t));
        return {a.x + t * dx, a.y + t * dy};
    }

    SnapInfo findNearestGraphPoint(double x, double y) const {
        SnapInfo best;
        for (const auto& edge : edges_) {
            const XYPoint& a = nodes_.at(edge.start_id);
            const XYPoint& b = nodes_.at(edge.end_id);
            double t = 0.0;
            const XYPoint projected = projectPointToSegment(x, y, a, b, t);
            const double dist = hypot2d(projected.x, projected.y, x, y);
            if (dist < best.dist) {
                best.point = projected;
                best.dist = dist;
                best.start_id = edge.start_id;
                best.end_id = edge.end_id;
                best.t = t;
                best.edge_len = edge.length;
                best.forward_allowed = edge.forward_allowed;
                best.backward_allowed = edge.backward_allowed;
                best.valid = true;
            }
        }

        if (!best.valid) {
            const auto nearest = findNearestNode(x, y);
            if (nearest.first.empty()) {
                return best;
            }
            best.point = nodes_.at(nearest.first);
            best.dist = nearest.second;
            best.start_id = nearest.first;
            best.end_id = nearest.first;
            best.t = 0.0;
            best.edge_len = 0.0;
            best.forward_allowed = true;
            best.backward_allowed = true;
            best.valid = true;
        }
        return best;
    }

    std::vector<std::string> reconstructNodePath(
        const std::unordered_map<std::string, std::string>& came_from,
        const std::string& goal_id) const {
        std::vector<std::string> path_ids{goal_id};
        std::string current = goal_id;
        while (came_from.find(current) != came_from.end()) {
            current = came_from.at(current);
            path_ids.push_back(current);
        }
        std::reverse(path_ids.begin(), path_ids.end());
        return path_ids;
    }

    std::pair<std::vector<std::string>, double> shortestNodePath(
        const std::string& start_id,
        const std::string& goal_id) const {
        using QueueItem = std::pair<double, std::string>;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open_set;
        std::unordered_map<std::string, std::string> came_from;
        std::unordered_map<std::string, double> g_score;
        open_set.push({0.0, start_id});
        g_score[start_id] = 0.0;

        while (!open_set.empty()) {
            const std::string current = open_set.top().second;
            open_set.pop();
            if (current == goal_id) {
                break;
            }

            auto found = adjacency_.find(current);
            if (found == adjacency_.end()) {
                continue;
            }

            for (const auto& neighbor : found->second) {
                const double tentative_g = g_score[current] + neighbor.second;
                if (g_score.find(neighbor.first) == g_score.end() || tentative_g < g_score[neighbor.first]) {
                    came_from[neighbor.first] = current;
                    g_score[neighbor.first] = tentative_g;
                    const XYPoint& goal = nodes_.at(goal_id);
                    const XYPoint& node = nodes_.at(neighbor.first);
                    const double h = hypot2d(goal.x, goal.y, node.x, node.y);
                    open_set.push({tentative_g + h, neighbor.first});
                }
            }
        }

        if (goal_id != start_id && came_from.find(goal_id) == came_from.end()) {
            return {{}, std::numeric_limits<double>::infinity()};
        }
        const double total_cost = (g_score.find(goal_id) != g_score.end()) ? g_score[goal_id] : 0.0;
        return {reconstructNodePath(came_from, goal_id), total_cost};
    }

    std::vector<XYPoint> dedupePathPoints(const std::vector<XYPoint>& path, double min_spacing = 1e-3) const {
        std::vector<XYPoint> compact;
        for (const auto& point : path) {
            if (compact.empty()) {
                compact.push_back(point);
                continue;
            }
            if (hypot2d(point.x, point.y, compact.back().x, compact.back().y) > min_spacing) {
                compact.push_back(point);
            }
        }
        return compact;
    }

    std::vector<XYPoint> buildGraphPath(const SnapInfo& start_snap, const SnapInfo& goal_snap) const {
        const XYPoint& start = start_snap.point;
        const XYPoint& goal = goal_snap.point;

        std::vector<XYPoint> best_path;
        double best_cost = std::numeric_limits<double>::infinity();

        if (start_snap.start_id == goal_snap.start_id && start_snap.end_id == goal_snap.end_id) {
            const bool direction_allowed =
                (goal_snap.t >= start_snap.t && start_snap.forward_allowed) ||
                (goal_snap.t < start_snap.t && start_snap.backward_allowed);
            if (direction_allowed) {
                best_path = {start, goal};
                best_cost = hypot2d(start.x, start.y, goal.x, goal.y);
            }
        }

        std::vector<std::pair<std::string, double>> start_candidates;
        std::vector<std::pair<std::string, double>> goal_candidates;
        if (start_snap.backward_allowed) {
            start_candidates.push_back({start_snap.start_id, start_snap.t * start_snap.edge_len});
        }
        if (start_snap.forward_allowed) {
            start_candidates.push_back({start_snap.end_id, (1.0 - start_snap.t) * start_snap.edge_len});
        }
        if (goal_snap.forward_allowed) {
            goal_candidates.push_back({goal_snap.start_id, goal_snap.t * goal_snap.edge_len});
        }
        if (goal_snap.backward_allowed) {
            goal_candidates.push_back({goal_snap.end_id, (1.0 - goal_snap.t) * goal_snap.edge_len});
        }

        for (const auto& start_candidate : start_candidates) {
            for (const auto& goal_candidate : goal_candidates) {
                const auto result = shortestNodePath(start_candidate.first, goal_candidate.first);
                if (result.first.empty()) {
                    continue;
                }

                const double total_cost = start_candidate.second + result.second + goal_candidate.second;
                std::vector<XYPoint> candidate;
                candidate.push_back(start);
                for (const auto& node_id : result.first) {
                    candidate.push_back(nodes_.at(node_id));
                }
                candidate.push_back(goal);
                candidate = dedupePathPoints(candidate);

                if (total_cost < best_cost && candidate.size() >= 2) {
                    best_cost = total_cost;
                    best_path = candidate;
                }
            }
        }

        return dedupePathPoints(best_path);
    }

    double rightLaneOffsetAt(const XYPoint& point) const {
        double nearest_dist = std::numeric_limits<double>::infinity();
        double road_width = default_road_width_;
        int lanes = 0;
        bool one_way = false;
        for (const auto& edge : edges_) {
            const XYPoint& a = nodes_.at(edge.start_id);
            const XYPoint& b = nodes_.at(edge.end_id);
            double t = 0.0;
            const XYPoint projected = projectPointToSegment(point.x, point.y, a, b, t);
            const double dist = hypot2d(point.x, point.y, projected.x, projected.y);
            if (dist < nearest_dist) {
                nearest_dist = dist;
                road_width = edge.road_width;
                lanes = edge.lanes;
                one_way = edge.forward_allowed != edge.backward_allowed;
            }
        }
        if (one_way && lanes <= 1) {
            return 0.0;
        }
        return std::min(max_right_lane_offset_, std::max(0.0, road_width * right_lane_offset_ratio_));
    }

    std::vector<XYPoint> applyRightLaneOffset(const std::vector<XYPoint>& center_path) const {
        if (!enable_right_lane_offset_ || center_path.size() < 2 || max_right_lane_offset_ <= 0.0) {
            return center_path;
        }

        std::vector<XYPoint> shifted;
        shifted.reserve(center_path.size());
        for (size_t i = 0; i < center_path.size(); ++i) {
            const XYPoint& prev = center_path[i == 0 ? 0 : i - 1];
            const XYPoint& next = center_path[i + 1 < center_path.size() ? i + 1 : i];
            const double dx = next.x - prev.x;
            const double dy = next.y - prev.y;
            const double length = std::hypot(dx, dy);
            if (length <= 1e-6) {
                shifted.push_back(center_path[i]);
                continue;
            }
            const double offset = rightLaneOffsetAt(center_path[i]);
            shifted.push_back({
                center_path[i].x + offset * dy / length,
                center_path[i].y - offset * dx / length
            });
        }
        return dedupePathPoints(shifted);
    }

    void publishInitialpose() {
        geometry_msgs::PoseWithCovarianceStamped init;
        init.header.frame_id = "map";
        init.header.stamp = ros::Time::now();
        init.pose.pose.position.x = robot_x_;
        init.pose.pose.position.y = robot_y_;
        init.pose.pose.orientation = quaternionFromYaw(robot_yaw_);
        for (double& value : init.pose.covariance) {
            value = 0.0;
        }
        init.pose.covariance[0] = initialpose_cov_xy_;
        init.pose.covariance[7] = initialpose_cov_xy_;
        init.pose.covariance[14] = 99999.0;
        init.pose.covariance[21] = 99999.0;
        init.pose.covariance[28] = 99999.0;
        init.pose.covariance[35] = initialpose_cov_yaw_;
        initialpose_pub_.publish(init);
        initialpose_sent_ = true;
    }

    void publishMarkerDelete(const std::string& ns, int id) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub_.publish(marker);
    }

    void publishPoseMarkers(bool has_start, const XYPoint& start_xy) {
        if (!publish_pose_markers_) {
            return;
        }

        publishMarkerDelete("osm_pose", 1001);

        if (has_start) {
            visualization_msgs::Marker start_marker;
            start_marker.header.frame_id = "map";
            start_marker.header.stamp = ros::Time::now();
            start_marker.ns = "osm_path";
            start_marker.id = 1001;
            start_marker.type = visualization_msgs::Marker::SPHERE;
            start_marker.action = visualization_msgs::Marker::ADD;
            start_marker.pose.position = toRosPoint(start_xy, 0.5);
            start_marker.pose.orientation.w = 1.0;
            start_marker.scale.x = path_endpoint_marker_scale_;
            start_marker.scale.y = path_endpoint_marker_scale_;
            start_marker.scale.z = path_endpoint_marker_scale_;
            start_marker.color.r = 0.10;
            start_marker.color.g = 0.70;
            start_marker.color.b = 1.0;
            start_marker.color.a = 0.95;
            marker_pub_.publish(start_marker);
        } else {
            publishMarkerDelete("osm_path", 1001);
            publishMarkerDelete("osm_path", 1004);
            publishMarkerDelete("osm_path", 1005);
        }

        if (!current_path_.empty()) {
            visualization_msgs::Marker path_points_marker;
            path_points_marker.header.frame_id = "map";
            path_points_marker.header.stamp = ros::Time::now();
            path_points_marker.ns = "osm_path";
            path_points_marker.id = 1004;
            path_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
            path_points_marker.action = visualization_msgs::Marker::ADD;
            path_points_marker.pose.orientation.w = 1.0;
            path_points_marker.scale.x = path_point_marker_scale_;
            path_points_marker.scale.y = path_point_marker_scale_;
            path_points_marker.scale.z = path_point_marker_scale_;
            path_points_marker.color.r = 1.0;
            path_points_marker.color.g = 0.55;
            path_points_marker.color.b = 0.05;
            path_points_marker.color.a = 0.92;
            for (const auto& point : current_path_) {
                path_points_marker.points.push_back(toRosPoint(point, 0.55));
            }
            marker_pub_.publish(path_points_marker);

            visualization_msgs::Marker path_line_marker;
            path_line_marker.header.frame_id = "map";
            path_line_marker.header.stamp = ros::Time::now();
            path_line_marker.ns = "osm_path";
            path_line_marker.id = 1005;
            path_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            path_line_marker.action = visualization_msgs::Marker::ADD;
            path_line_marker.pose.orientation.w = 1.0;
            path_line_marker.scale.x = path_line_marker_width_;
            path_line_marker.color.r = 1.0;
            path_line_marker.color.g = 0.35;
            path_line_marker.color.b = 0.10;
            path_line_marker.color.a = 0.90;
            for (const auto& point : current_path_) {
                path_line_marker.points.push_back(toRosPoint(point, 0.45));
            }
            marker_pub_.publish(path_line_marker);
        }

        visualization_msgs::Marker current_marker;
        current_marker.header.frame_id = "map";
        current_marker.header.stamp = ros::Time::now();
        current_marker.ns = "osm_pose";
        current_marker.id = 1002;
        current_marker.type = visualization_msgs::Marker::ARROW;
        current_marker.action = visualization_msgs::Marker::ADD;
        current_marker.pose.position = toRosPoint({robot_x_, robot_y_}, 0.4);
        current_marker.pose.orientation = quaternionFromYaw(robot_yaw_);
        current_marker.scale.x = 1.6;
        current_marker.scale.y = 0.35;
        current_marker.scale.z = 0.35;
        current_marker.color.r = 0.05;
        current_marker.color.g = 0.95;
        current_marker.color.b = 0.15;
        current_marker.color.a = 0.95;
        marker_pub_.publish(current_marker);

        if (has_gps_ && !last_gps_recv_.isZero()) {
            const double gps_age = (ros::Time::now() - last_gps_recv_).toSec();
            const bool gps_fresh = gps_age <= gps_marker_timeout_;

            visualization_msgs::Marker gps_marker;
            gps_marker.header.frame_id = "map";
            gps_marker.header.stamp = ros::Time::now();
            gps_marker.ns = "osm_roads";
            gps_marker.id = 1003;
            gps_marker.type = visualization_msgs::Marker::SPHERE;
            gps_marker.action = visualization_msgs::Marker::ADD;
            gps_marker.pose.position = toRosPoint({gps_x_, gps_y_}, 0.6);
            gps_marker.pose.orientation.w = 1.0;
            gps_marker.scale.x = 0.8;
            gps_marker.scale.y = 0.8;
            gps_marker.scale.z = 0.8;
            gps_marker.color.r = 1.0;
            gps_marker.color.g = 0.92;
            gps_marker.color.b = 0.05;
            gps_marker.color.a = gps_fresh ? 0.95 : 0.35;
            marker_pub_.publish(gps_marker);
        }

        publishCurrentSnapMarker();
    }

    void publishSnapDebugMarkers(
        bool has_raw_start,
        const XYPoint& raw_start,
        bool has_snap_start,
        const XYPoint& snap_start,
        bool has_raw_goal,
        const XYPoint& raw_goal,
        bool has_snap_goal,
        const XYPoint& snap_goal) {
        if (!publish_snap_debug_markers_) {
            return;
        }

        visualization_msgs::Marker start_marker;
        start_marker.header.frame_id = "map";
        start_marker.header.stamp = ros::Time::now();
        start_marker.ns = "osm_snap";
        start_marker.id = 1101;
        start_marker.type = visualization_msgs::Marker::SPHERE;
        start_marker.action = has_snap_start ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
        if (has_snap_start) {
            start_marker.pose.position = toRosPoint(snap_start, 0.7);
            start_marker.pose.orientation.w = 1.0;
            start_marker.scale.x = snap_marker_scale_;
            start_marker.scale.y = snap_marker_scale_;
            start_marker.scale.z = snap_marker_scale_;
            start_marker.color.g = 1.0;
            start_marker.color.b = 1.0;
            start_marker.color.a = 0.9;
        }
        marker_pub_.publish(start_marker);

        visualization_msgs::Marker goal_marker;
        goal_marker.header.frame_id = "map";
        goal_marker.header.stamp = ros::Time::now();
        goal_marker.ns = "osm_snap";
        goal_marker.id = 1102;
        goal_marker.type = visualization_msgs::Marker::SPHERE;
        goal_marker.action = has_snap_goal ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
        if (has_snap_goal) {
            goal_marker.pose.position = toRosPoint(snap_goal, 0.7);
            goal_marker.pose.orientation.w = 1.0;
            goal_marker.scale.x = snap_marker_scale_;
            goal_marker.scale.y = snap_marker_scale_;
            goal_marker.scale.z = snap_marker_scale_;
            goal_marker.color.r = 1.0;
            goal_marker.color.b = 1.0;
            goal_marker.color.a = 0.9;
        }
        marker_pub_.publish(goal_marker);

        visualization_msgs::Marker start_link;
        start_link.header.frame_id = "map";
        start_link.header.stamp = ros::Time::now();
        start_link.ns = "osm_snap";
        start_link.id = 1103;
        start_link.type = visualization_msgs::Marker::LINE_LIST;
        start_link.action = (has_raw_start && has_snap_start) ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
        if (start_link.action == visualization_msgs::Marker::ADD) {
            start_link.scale.x = snap_link_marker_width_;
            start_link.color.g = 1.0;
            start_link.color.b = 1.0;
            start_link.color.a = 0.85;
            start_link.points = {toRosPoint(raw_start, 0.35), toRosPoint(snap_start, 0.35)};
        }
        marker_pub_.publish(start_link);

        visualization_msgs::Marker goal_link;
        goal_link.header.frame_id = "map";
        goal_link.header.stamp = ros::Time::now();
        goal_link.ns = "osm_snap";
        goal_link.id = 1104;
        goal_link.type = visualization_msgs::Marker::LINE_LIST;
        goal_link.action = (has_raw_goal && has_snap_goal) ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
        if (goal_link.action == visualization_msgs::Marker::ADD) {
            goal_link.scale.x = snap_link_marker_width_;
            goal_link.color.r = 1.0;
            goal_link.color.b = 1.0;
            goal_link.color.a = 0.85;
            goal_link.points = {toRosPoint(raw_goal, 0.35), toRosPoint(snap_goal, 0.35)};
        }
        marker_pub_.publish(goal_link);
    }

    void publishCurrentSnapMarker() {
        if (!publish_snap_debug_markers_) {
            return;
        }

        visualization_msgs::Marker snap_marker;
        snap_marker.header.frame_id = "map";
        snap_marker.header.stamp = ros::Time::now();
        snap_marker.ns = "osm_snap";
        snap_marker.id = 1105;
        snap_marker.type = visualization_msgs::Marker::SPHERE;

        visualization_msgs::Marker link_marker;
        link_marker.header.frame_id = "map";
        link_marker.header.stamp = ros::Time::now();
        link_marker.ns = "osm_snap";
        link_marker.id = 1106;
        link_marker.type = visualization_msgs::Marker::LINE_LIST;

        if (nodes_.empty()) {
            snap_marker.action = visualization_msgs::Marker::DELETE;
            link_marker.action = visualization_msgs::Marker::DELETE;
            marker_pub_.publish(snap_marker);
            marker_pub_.publish(link_marker);
            return;
        }

        const SnapInfo snap_info = findNearestGraphPoint(robot_x_, robot_y_);
        if (!snap_info.valid) {
            snap_marker.action = visualization_msgs::Marker::DELETE;
            link_marker.action = visualization_msgs::Marker::DELETE;
            marker_pub_.publish(snap_marker);
            marker_pub_.publish(link_marker);
            return;
        }

        snap_marker.action = visualization_msgs::Marker::ADD;
        snap_marker.pose.position = toRosPoint(snap_info.point, 0.55);
        snap_marker.pose.orientation.w = 1.0;
        snap_marker.scale.x = snap_marker_scale_;
        snap_marker.scale.y = snap_marker_scale_;
        snap_marker.scale.z = snap_marker_scale_;
        snap_marker.color.r = 1.0;
        snap_marker.color.g = 0.55;
        snap_marker.color.a = 0.95;
        marker_pub_.publish(snap_marker);

        link_marker.action = visualization_msgs::Marker::ADD;
        link_marker.scale.x = snap_link_marker_width_;
        link_marker.color.r = 1.0;
        link_marker.color.g = 0.55;
        link_marker.color.a = 0.85;
        link_marker.points = {
            toRosPoint({robot_x_, robot_y_}, 0.30),
            toRosPoint(snap_info.point, 0.30)
        };
        marker_pub_.publish(link_marker);
    }

    double computeTurnAngle(const XYPoint& prev, const XYPoint& curr, const XYPoint& next) const {
        const double v1x = curr.x - prev.x;
        const double v1y = curr.y - prev.y;
        const double v2x = next.x - curr.x;
        const double v2y = next.y - curr.y;
        const double norm1 = std::hypot(v1x, v1y);
        const double norm2 = std::hypot(v2x, v2y);
        if (norm1 <= 1e-6 || norm2 <= 1e-6) {
            return 0.0;
        }
        double cos_theta = (v1x * v2x + v1y * v2y) / (norm1 * norm2);
        cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
        return std::acos(cos_theta);
    }

    double spacingFromTurnAngle(double angle_rad) const {
        if (angle_rad <= path_turn_angle_low_rad_) {
            return path_straight_spacing_;
        }
        if (angle_rad >= path_turn_angle_high_rad_) {
            return path_turn_spacing_;
        }
        const double ratio =
            (angle_rad - path_turn_angle_low_rad_) /
            std::max(1e-6, path_turn_angle_high_rad_ - path_turn_angle_low_rad_);
        return path_straight_spacing_ + ratio * (path_turn_spacing_ - path_straight_spacing_);
    }

    std::vector<double> computeCumulativeLengths(const std::vector<XYPoint>& path) const {
        std::vector<double> cumulative;
        cumulative.push_back(0.0);
        for (size_t i = 1; i < path.size(); ++i) {
            cumulative.push_back(cumulative.back() + hypot2d(path[i].x, path[i].y, path[i - 1].x, path[i - 1].y));
        }
        return cumulative;
    }

    XYPoint interpolateAlongPath(const std::vector<XYPoint>& path, const std::vector<double>& cumulative, double target_s) const {
        const double total_len = cumulative.back();
        if (target_s <= 0.0) {
            return path.front();
        }
        if (target_s >= total_len) {
            return path.back();
        }
        for (size_t i = 1; i < cumulative.size(); ++i) {
            if (target_s <= cumulative[i]) {
                const double seg_start = cumulative[i - 1];
                const double seg_len = cumulative[i] - seg_start;
                if (seg_len <= 1e-9) {
                    return path[i];
                }
                const double ratio = (target_s - seg_start) / seg_len;
                return {
                    path[i - 1].x + ratio * (path[i].x - path[i - 1].x),
                    path[i - 1].y + ratio * (path[i].y - path[i - 1].y)
                };
            }
        }
        return path.back();
    }

    struct TurnInfluence {
        double vertex_s;
        double dense_spacing;
    };

    std::vector<TurnInfluence> buildTurnInfluences(const std::vector<XYPoint>& path, const std::vector<double>& cumulative) const {
        std::vector<TurnInfluence> influences;
        if (path.size() <= 2) {
            return influences;
        }
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            const double angle = computeTurnAngle(path[i - 1], path[i], path[i + 1]);
            const double dense_spacing = spacingFromTurnAngle(angle);
            if (dense_spacing >= path_straight_spacing_ - 1e-6) {
                continue;
            }
            influences.push_back({cumulative[i], dense_spacing});
        }
        return influences;
    }

    double spacingAtArcLength(double arc_s, const std::vector<TurnInfluence>& influences) const {
        double spacing = path_straight_spacing_;
        for (const auto& influence : influences) {
            const double dense_start = influence.vertex_s - path_turn_dense_before_;
            const double dense_end = influence.vertex_s + path_turn_dense_after_;
            const double recover_end = dense_end + path_turn_recover_after_;

            double local_spacing = path_straight_spacing_;
            if (dense_start <= arc_s && arc_s <= dense_end) {
                local_spacing = influence.dense_spacing;
            } else if (dense_end < arc_s && arc_s < recover_end && path_turn_recover_after_ > 1e-6) {
                const double ratio = (arc_s - dense_end) / path_turn_recover_after_;
                local_spacing = influence.dense_spacing + ratio * (path_straight_spacing_ - influence.dense_spacing);
            }
            spacing = std::min(spacing, local_spacing);
        }
        return std::max(path_turn_spacing_, std::min(path_straight_spacing_, spacing));
    }

    std::vector<XYPoint> resamplePathForTracking(const std::vector<XYPoint>& path) const {
        std::vector<XYPoint> deduped = dedupePathPoints(path);
        if (deduped.size() < 2) {
            return deduped;
        }

        const std::vector<double> cumulative = computeCumulativeLengths(deduped);
        const double total_len = cumulative.back();
        if (total_len <= 1e-6) {
            return deduped;
        }

        const std::vector<TurnInfluence> influences = buildTurnInfluences(deduped, cumulative);
        std::vector<XYPoint> resampled;
        resampled.push_back(deduped.front());

        double arc_s = 0.0;
        while (arc_s < total_len) {
            const double step = std::max(1e-3, spacingAtArcLength(arc_s, influences));
            const double next_s = std::min(total_len, arc_s + step);
            if (next_s <= arc_s + 1e-6) {
                break;
            }
            const XYPoint point = interpolateAlongPath(deduped, cumulative, next_s);
            if (hypot2d(point.x, point.y, resampled.back().x, resampled.back().y) > 0.05) {
                resampled.push_back(point);
            }
            arc_s = next_s;
        }

        if (resampled.size() < static_cast<size_t>(path_min_points_) && resampled.size() >= 2) {
            std::vector<XYPoint> padded;
            for (int i = 0; i < path_min_points_; ++i) {
                const double t = (path_min_points_ == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(path_min_points_ - 1);
                padded.push_back({
                    resampled.front().x + t * (resampled.back().x - resampled.front().x),
                    resampled.front().y + t * (resampled.back().y - resampled.front().y)
                });
            }
            return padded;
        }
        return resampled;
    }

    nav_msgs::Path buildPathMessage(const std::vector<XYPoint>& path) const {
        nav_msgs::Path nav_path;
        nav_path.header.frame_id = "map";
        nav_path.header.stamp = ros::Time::now();
        double last_yaw = 0.0;
        bool has_last_yaw = false;
        for (size_t i = 0; i < path.size(); ++i) {
            geometry_msgs::PoseStamped pose;
            pose.header = nav_path.header;
            pose.pose.position = toRosPoint(path[i]);
            pose.pose.orientation.w = 1.0;
            if (i + 1 < path.size()) {
                last_yaw = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
                has_last_yaw = true;
                pose.pose.orientation = quaternionFromYaw(last_yaw);
            } else if (has_last_yaw) {
                // Keep the terminal heading aligned with the final path segment.
                pose.pose.orientation = quaternionFromYaw(last_yaw);
            }
            nav_path.poses.push_back(pose);
        }
        return nav_path;
    }

    bool prepareGoalRequest(const geometry_msgs::PoseStamped& goal) {
        if (!has_odom_ || nodes_.empty()) {
            std::vector<std::string> missing;
            if (!has_odom_) {
                missing.push_back("/odometry/global");
            }
            if (nodes_.empty()) {
                missing.push_back("OSM road graph");
            }
            std::string joined;
            for (size_t i = 0; i < missing.size(); ++i) {
                if (i > 0) {
                    joined += ", ";
                }
                joined += missing[i];
            }
            ROS_WARN_THROTTLE(1.0, "OSM C++ planner waiting for: %s", joined.c_str());
            return false;
        }

        if (!goal.header.frame_id.empty() && goal.header.frame_id != "map") {
            ROS_WARN("Goal frame is '%s', expected 'map'. Coordinates are used as-is.", goal.header.frame_id.c_str());
        }

        ROS_INFO("Goal accepted: (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
        if (!has_map_) {
            ROS_WARN_THROTTLE(1.0, "No /map yet. OSM graph path can still be generated, but occupancy map checks are unavailable.");
        }

        if (auto_set_initialpose_) {
            publishInitialpose();
        }
        return true;
    }

    void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
        goal.header.stamp = ros::Time::now();
        goal.pose.position = msg->point;
        goal.pose.orientation.w = 1.0;
        ROS_INFO("Clicked point received: (%.2f, %.2f) frame=%s", goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());
        if (prepareGoalRequest(goal)) {
            processGoal(goal);
        }
    }

    void processGoal(const geometry_msgs::PoseStamped& goal) {
        const double gx = goal.pose.position.x;
        const double gy = goal.pose.position.y;
        const SnapInfo start_snap = findNearestGraphPoint(robot_x_, robot_y_);
        const SnapInfo goal_snap = findNearestGraphPoint(gx, gy);
        const double start_dist = start_snap.valid ? start_snap.dist : std::numeric_limits<double>::infinity();
        const double goal_dist = goal_snap.valid ? goal_snap.dist : std::numeric_limits<double>::infinity();

        publishSnapDebugMarkers(
            true, {robot_x_, robot_y_},
            start_snap.valid, start_snap.point,
            true, {gx, gy},
            goal_snap.valid, goal_snap.point);

        ROS_INFO("Snap distance: start=%.2fm, goal=%.2fm", start_dist, goal_dist);

        if (!start_snap.valid || !goal_snap.valid) {
            ROS_WARN("OSM C++ planner cannot find nearest drivable graph point");
            return;
        }

        if (strict_road_mode_) {
            if (start_dist > start_snap_max_dist_) {
                ROS_WARN("Start is too far from drivable graph (%.2fm > %.2fm). Check GPS/TF alignment or road graph coverage.", start_dist, start_snap_max_dist_);
                return;
            }
            if (goal_dist > goal_snap_max_dist_) {
                ROS_WARN("Goal is too far from drivable graph (%.2fm > %.2fm). Please click near blue road lines.", goal_dist, goal_snap_max_dist_);
                return;
            }
        }

        std::vector<XYPoint> graph_path = buildGraphPath(start_snap, goal_snap);
        if (graph_path.empty()) {
            ROS_WARN("OSM graph A* failed to find a drivable path");
            return;
        }

        std::vector<XYPoint> raw_path = applyRightLaneOffset(resamplePathForTracking(graph_path));
        if (!strict_road_mode_) {
            raw_path.push_back({gx, gy});
            raw_path = dedupePathPoints(raw_path);
        }

        current_path_ = resamplePathForTracking(raw_path);
        if (current_path_.size() < 2) {
            ROS_WARN("OSM graph path degenerated to %zu point(s); rejecting this request.", current_path_.size());
            return;
        }

        path_pub_.publish(buildPathMessage(current_path_));
        publishPoseMarkers(true, current_path_.front());
        ROS_INFO(
            "Road path planned: graph=%zu points, tracking=%zu points, start snap %.2fm, goal snap %.2fm, right lane offset=%s",
            graph_path.size(),
            current_path_.size(),
            start_dist,
            goal_dist,
            enable_right_lane_offset_ ? "enabled" : "disabled");
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!prepareGoalRequest(*msg)) {
            return;
        }
        processGoal(*msg);
    }

    void timerCallback(const ros::TimerEvent&) {
        if (!has_odom_) {
            return;
        }

        if (auto_set_initialpose_ && !initialpose_sent_ && has_map_) {
            publishInitialpose();
        }

        if (current_path_.empty()) {
            publishPoseMarkers(false, {});
            return;
        }

        while (current_path_.size() > 1) {
            if (hypot2d(current_path_.front().x, current_path_.front().y, robot_x_, robot_y_) < std::max(0.8, lookahead_dist_ * 0.5)) {
                current_path_.erase(current_path_.begin());
            } else {
                break;
            }
        }

        size_t target_idx = current_path_.size() - 1;
        double accum = 0.0;
        double prev_x = robot_x_;
        double prev_y = robot_y_;
        for (size_t i = 0; i < current_path_.size(); ++i) {
            accum += hypot2d(current_path_[i].x, current_path_[i].y, prev_x, prev_y);
            if (accum >= lookahead_dist_) {
                target_idx = i;
                break;
            }
            prev_x = current_path_[i].x;
            prev_y = current_path_[i].y;
        }

        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.pose.position = toRosPoint(current_path_[target_idx]);

        double yaw = robot_yaw_;
        if (target_idx + 1 < current_path_.size()) {
            yaw = std::atan2(
                current_path_[target_idx + 1].y - current_path_[target_idx].y,
                current_path_[target_idx + 1].x - current_path_[target_idx].x);
        }
        goal_msg.pose.orientation = quaternionFromYaw(yaw);
        subgoal_pub_.publish(goal_msg);

        publishPoseMarkers(true, current_path_.front());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_astar_planner");
    OSMGraphAStarPlanner planner;
    ros::spin();
    return 0;
}
