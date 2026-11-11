// Load plugins only. No initialize(), costmap, sensor, actuator or ROS node handle.
#include <iostream>
#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>

int main(int argc,char** argv) {
  ros::init(argc,argv,"experiment_plugin_load_only",ros::init_options::AnonymousName|ros::init_options::NoSigintHandler);
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> global("nav_core","nav_core::BaseGlobalPlanner");
  auto original=global.createInstance("teb_test/OSMPathPlanner");
  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> local("nav_core","nav_core::BaseLocalPlanner");
  auto teb=local.createInstance("teb_local_planner/TebLocalPlannerROS");
  std::cout<<"TEB_PLUGIN_PASS: recovered OSM planner and installed TEB load; no planners initialized"<<std::endl;
  return original && teb ? 0 : 1;
}
