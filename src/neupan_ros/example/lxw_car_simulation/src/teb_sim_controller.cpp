// Simulation adapter for the unmodified, installed ROS navigation TEB plugin.
// Both planners receive the same reference path. No move_base/global planner is
// introduced into the comparison, and angular.z is explicitly a steering angle.
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "teb_sim_controller");
  ros::NodeHandle nh, priv("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener listener(buffer);
  costmap_2d::Costmap2DROS costmap("local_costmap", buffer);
  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> loader("nav_core", "nav_core::BaseLocalPlanner");
  auto planner = loader.createInstance("teb_local_planner/TebLocalPlannerROS");
  planner->initialize("TebLocalPlannerROS", &buffer, &costmap);

  std::mutex mutex;
  std::vector<geometry_msgs::PoseStamped> pending;
  bool new_plan = false, active = false, arrived = false;
  ros::Time scan_stamp;
  auto cmd_pub = nh.advertise<geometry_msgs::Twist>("/planner_sim/raw_cmd", 1);
  auto status_pub = nh.advertise<std_msgs::String>("/planner_sim/runtime", 10);
  auto arrival_pub = nh.advertise<std_msgs::String>("/goal_arrived", 1, true);
  auto ready_pub = nh.advertise<std_msgs::Bool>("/planner_sim/controller_ready", 1, true);
  auto common_plan_pub = nh.advertise<nav_msgs::Path>("/planner_sim/local_plan", 1);
  auto local_plan_sub = priv.subscribe<nav_msgs::Path>("TebLocalPlannerROS/local_plan", 1,
    [&](const nav_msgs::Path::ConstPtr& msg) { common_plan_pub.publish(*msg); });
  auto path_sub = nh.subscribe<nav_msgs::Path>("/initial_path", 1,
    [&](const nav_msgs::Path::ConstPtr& msg) {
      if (msg->header.frame_id != "odom" || msg->poses.size() < 2) {
        ROS_ERROR("TEB simulation requires a nonempty odom reference path");
        return;
      }
      std::lock_guard<std::mutex> lock(mutex);
      pending = msg->poses;
      new_plan = true;
    });
  auto scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1,
    [&](const sensor_msgs::LaserScan::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(mutex);
      scan_stamp = msg->header.stamp;
    });
  std_msgs::Bool ready; ready.data = true; ready_pub.publish(ready);
  ros::Rate rate(10.0);
  while (ros::ok()) {
    bool fresh_scan;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (new_plan) {
        active = planner->setPlan(pending);
        arrived = false;
        new_plan = false;
      }
      const double age = (ros::Time::now() - scan_stamp).toSec();
      fresh_scan = !scan_stamp.isZero() && age >= 0.0 && age < 0.5;
    }
    geometry_msgs::Twist cmd;
    bool valid = false;
    const auto start = std::chrono::steady_clock::now();
    const bool attempted = active && fresh_scan && costmap.isCurrent();
    if (attempted) {
      boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap.getCostmap()->getMutex());
      valid = planner->computeVelocityCommands(cmd);
      arrived = valid && planner->isGoalReached();
      if (arrived) active = false;
    }
    const double elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    if (!valid || arrived || !std::isfinite(cmd.linear.x) || !std::isfinite(cmd.angular.z))
      cmd = geometry_msgs::Twist();
    cmd_pub.publish(cmd);
    std_msgs::String arrival; arrival.data = arrived ? "1" : "0"; arrival_pub.publish(arrival);
    std::ostringstream json;
    json << "{\"solver_ms\":" << elapsed << ",\"valid\":" << (valid ? "true" : "false")
         << ",\"attempted\":" << (attempted ? "true" : "false")
         << ",\"planner_arrive\":" << (arrived ? "true" : "false")
         << ",\"scan_fresh\":" << (fresh_scan ? "true" : "false") << "}";
    std_msgs::String status; status.data = json.str(); status_pub.publish(status);
    rate.sleep();
  }
  cmd_pub.publish(geometry_msgs::Twist());
}
