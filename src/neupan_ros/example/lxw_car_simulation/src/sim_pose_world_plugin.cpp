#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_msgs/ModelState.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>

namespace gazebo {
// Simulation-only transport for the existing kinematic pose driver. Applying
// commands on the physics update thread avoids SetModelState's world pause.
class LxwSimPose : public WorldPlugin {
 public:
  void Load(physics::WorldPtr world, sdf::ElementPtr) override {
    if (!ros::isInitialized()) {
      gzerr << "LxwSimPose requires gazebo_ros_api_plugin\n";
      return;
    }
    world_ = world;
    node_.reset(new ros::NodeHandle());
    node_->setCallbackQueue(&queue_);
    sub_ = node_->subscribe("/planner_sim/model_state", 20, &LxwSimPose::receive,
                           this, ros::TransportHints().tcpNoDelay());
    spinner_.reset(new ros::AsyncSpinner(1, &queue_));
    spinner_->start();
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&LxwSimPose::apply, this));
    ROS_INFO("LXW simulation pose transport ready (world update, no pause/unpause)");
  }

  ~LxwSimPose() override {
    update_.reset();
    if (spinner_) spinner_->stop();
    if (node_) node_->shutdown();
  }

 private:
  void receive(const gazebo_msgs::ModelState::ConstPtr &msg) {
    if (msg->reference_frame != "world" && !msg->reference_frame.empty()) {
      ROS_ERROR_THROTTLE(1., "LXW pose transport accepts only world-frame poses");
      return;
    }
    const auto &p = msg->pose;
    if (!(std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
          std::isfinite(p.position.z) && std::isfinite(p.orientation.w) &&
          std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) &&
          std::isfinite(p.orientation.z))) return;
    std::lock_guard<std::mutex> guard(mutex_);
    pending_[msg->model_name] = *msg;
  }

  void apply() {
    std::map<std::string, gazebo_msgs::ModelState> commands;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      commands.swap(pending_);
    }
    for (const auto &entry : commands) {
      const auto model = world_->ModelByName(entry.first);
      if (!model) continue;
      const auto &msg = entry.second;
      const auto &p = msg.pose;
      ignition::math::Quaterniond rotation(p.orientation.w, p.orientation.x,
                                           p.orientation.y, p.orientation.z);
      const double norm2 = p.orientation.w*p.orientation.w + p.orientation.x*p.orientation.x +
                           p.orientation.y*p.orientation.y + p.orientation.z*p.orientation.z;
      if (!std::isfinite(norm2) || norm2 < 1e-12) continue;
      rotation.Normalize();
      model->SetWorldPose(ignition::math::Pose3d(
          ignition::math::Vector3d(p.position.x, p.position.y, p.position.z), rotation));
      model->SetLinearVel(ignition::math::Vector3d(
          msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z));
      model->SetAngularVel(ignition::math::Vector3d(
          msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z));
    }
  }

  physics::WorldPtr world_;
  event::ConnectionPtr update_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::CallbackQueue queue_;
  ros::Subscriber sub_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  std::mutex mutex_;
  std::map<std::string, gazebo_msgs::ModelState> pending_;
};
GZ_REGISTER_WORLD_PLUGIN(LxwSimPose)
}  // namespace gazebo
