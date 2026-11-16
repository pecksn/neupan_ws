// Module contract: Experimental move_base global-planner plugin consuming the OSM
// path. This adapter is opt-in and does not run the NeuPAN local solver.

#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace teb_test {

// Adapt retained OSM reference geometry to the nav_core global-planner interface.
class OSMPathPlanner : public nav_core::BaseGlobalPlanner {
  public:
    OSMPathPlanner() = default;

    // Adapt retained OSM reference geometry to the nav_core global-planner interface.
    OSMPathPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
        initialize(name, costmap_ros);
    }

    // Configure plugin topics/parameters and register path input once for this planner
    // instance.
    void initialize(std::string name,
                    costmap_2d::Costmap2DROS * /*costmap_ros*/) override {
        if (initialized_) {
            return;
        }

        ros::NodeHandle private_nh("~/" + name);
        private_nh.param<std::string>("path_topic", path_topic_, "/osm_global_path");
        private_nh.param<std::string>("goal_topic", goal_topic_, "/teb_test/osm_goal");
        private_nh.param<std::string>("expected_frame", expected_frame_, "map");
        private_nh.param("plan_wait_timeout", plan_wait_timeout_, 2.0);
        private_nh.param("goal_match_tolerance", goal_match_tolerance_, 8.0);
        private_nh.param("debug_log", debug_log_, true);

        ros::NodeHandle nh;
        path_sub_ = nh.subscribe(path_topic_, 1, &OSMPathPlanner::pathCallback, this);
        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);

        initialized_ = true;
        ROS_INFO("OSMPathPlanner initialized: path_topic=%s, goal_topic=%s, "
                 "expected_frame=%s",
                 path_topic_.c_str(), goal_topic_.c_str(), expected_frame_.c_str());
    }

    // Build a move_base plan from the eligible retained OSM reference and requested
    // start/goal context.
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan) override {
        plan.clear();

        if (!initialized_) {
            ROS_ERROR("OSMPathPlanner used before initialize()");
            return false;
        }

        if (!expected_frame_.empty() && !goal.header.frame_id.empty() &&
            goal.header.frame_id != expected_frame_) {
            ROS_WARN_THROTTLE(
                1.0, "OSMPathPlanner received goal in frame '%s', expected '%s'",
                goal.header.frame_id.c_str(), expected_frame_.c_str());
        }

        const ros::Time request_time = ros::Time::now();
        if (debug_log_) {
            ROS_INFO("OSMPathPlanner makePlan: start=(%.2f, %.2f) goal=(%.2f, %.2f) "
                     "frame=%s via=%s",
                     start.pose.position.x, start.pose.position.y, goal.pose.position.x,
                     goal.pose.position.y, goal.header.frame_id.c_str(),
                     goal_topic_.c_str());
        }
        goal_pub_.publish(goal);

        ros::Rate rate(20.0);
        const ros::Time deadline = request_time + ros::Duration(plan_wait_timeout_);
        while (ros::ok() && ros::Time::now() < deadline) {
            std::vector<geometry_msgs::PoseStamped> candidate;
            ros::Time path_stamp;
            {
                std::lock_guard<std::mutex> lock(path_mutex_);
                candidate = latest_plan_;
                path_stamp = latest_path_stamp_;
            }

            if (isUsablePlan(candidate, path_stamp, request_time, start, goal)) {
                plan = candidate;
                if (debug_log_) {
                    const double start_err = std::hypot(
                        candidate.front().pose.position.x - start.pose.position.x,
                        candidate.front().pose.position.y - start.pose.position.y);
                    const double goal_err = std::hypot(
                        candidate.back().pose.position.x - goal.pose.position.x,
                        candidate.back().pose.position.y - goal.pose.position.y);
                    ROS_INFO("OSMPathPlanner accepted plan: points=%zu start_err=%.2f "
                             "goal_err=%.2f",
                             plan.size(), start_err, goal_err);
                }
                return true;
            }

            rate.sleep();
        }

        if (debug_log_) {
            std::ostringstream oss;
            oss << "OSMPathPlanner timeout after " << plan_wait_timeout_
                << "s waiting for matching /osm_global_path";
            std::lock_guard<std::mutex> lock(path_mutex_);
            oss << " latest_points=" << latest_plan_.size();
            if (!latest_plan_.empty()) {
                const auto &s = latest_plan_.front().pose.position;
                const auto &g = latest_plan_.back().pose.position;
                oss << " latest_start=(" << s.x << "," << s.y << ")";
                oss << " latest_goal=(" << g.x << "," << g.y << ")";
            }
            ROS_WARN("%s", oss.str().c_str());
        } else {
            ROS_WARN("OSMPathPlanner timed out waiting %.2fs for a matching "
                     "/osm_global_path",
                     plan_wait_timeout_);
        }
        return false;
    }

  private:
    // Retain a newly received OSM path with the synchronization used by makePlan.
    void pathCallback(const nav_msgs::Path::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        latest_plan_ = msg->poses;
        latest_path_stamp_ = ros::Time::now();
    }

    // Check the retained reference's frame, geometry, and goal association before
    // returning it as a usable plan.
    bool isUsablePlan(const std::vector<geometry_msgs::PoseStamped> &candidate,
                      const ros::Time &path_stamp, const ros::Time &request_time,
                      const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal) const {
        if (candidate.size() < 2) {
            return false;
        }

        if (path_stamp < request_time) {
            return false;
        }

        const auto &plan_start = candidate.front().pose.position;
        const auto &plan_goal = candidate.back().pose.position;
        const auto &start_pos = start.pose.position;
        const auto &goal_pos = goal.pose.position;

        const double start_dist =
            std::hypot(plan_start.x - start_pos.x, plan_start.y - start_pos.y);
        const double goal_dist =
            std::hypot(plan_goal.x - goal_pos.x, plan_goal.y - goal_pos.y);

        return start_dist <= goal_match_tolerance_ &&
               goal_dist <= goal_match_tolerance_;
    }

    bool initialized_{false};
    std::string path_topic_;
    std::string goal_topic_;
    std::string expected_frame_;
    double plan_wait_timeout_{2.0};
    double goal_match_tolerance_{8.0};
    bool debug_log_{true};

    ros::Subscriber path_sub_;
    ros::Publisher goal_pub_;

    mutable std::mutex path_mutex_;
    std::vector<geometry_msgs::PoseStamped> latest_plan_;
    ros::Time latest_path_stamp_;
};

} // namespace teb_test

PLUGINLIB_EXPORT_CLASS(teb_test::OSMPathPlanner, nav_core::BaseGlobalPlanner)
