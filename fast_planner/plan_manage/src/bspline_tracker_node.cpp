/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include "bspline/non_uniform_bspline.h"
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "fast_planner_msgs/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"


namespace fast_planner {

class BsplineTrackerNode : public rclcpp::Node {
 public:
  BsplineTrackerNode(const rclcpp::NodeOptions &options)
      : Node("bspline_tracker_node", options), receive_traj_(false) {
    init();
  }

  ~BsplineTrackerNode() {}

 private:
  void init();
  void bsplineCallback(const fast_planner_msgs::msg::Bspline::SharedPtr msg);
  void replanCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void newCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void PosCmdTimerCallback();
  void VisCmdTimerCallback();

  void displayTrajWithColor(const std::vector<Eigen::Vector3d>& path, double resolution,
                        const Eigen::Vector4d& color, int id); 

  void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
               const Eigen::Vector4d& color);

  rclcpp::Subscription<fast_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr replan_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr new_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr pos_cmd_timer_, vis_cmd_timer_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vis_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;

  nav_msgs::msg::Odometry odom;

  quadrotor_msgs::msg::PositionCommand cmd;
  double pos_gain[3] = {5.7, 5.7, 6.2};
  double vel_gain[3] = {3.4, 3.4, 4.0};

  bool receive_traj_;
  vector<NonUniformBspline> traj_;
  double traj_duration_;
  rclcpp::Time start_time_;
  int traj_id_;

  // yaw control
  double last_yaw_;
  double time_forward_;

  vector<Eigen::Vector3d> traj_cmd_, traj_real_;
};

void BsplineTrackerNode::init() {

  // control parameter 
  /*cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];
  */

  // Get BsplineTrackerNode parameters
  this->declare_parameter("bspline_tracker.time_forward", -1.0);
  this->get_parameter("bspline_tracker.time_forward", time_forward_);
  RCLCPP_INFO(this->get_logger(), "bspline_tracker: time_forward: %f", time_forward_);

  // ROS2 subscribers
  bspline_sub_ = this->create_subscription<fast_planner_msgs::msg::Bspline>(
      "planning/bspline", 10, std::bind(&BsplineTrackerNode::bsplineCallback, this, std::placeholders::_1));
  replan_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "planning/replan", 10, std::bind(&BsplineTrackerNode::replanCallback, this, std::placeholders::_1));
  new_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "planning/new", 10, std::bind(&BsplineTrackerNode::newCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&BsplineTrackerNode::odomCallback, this, std::placeholders::_1));

  // ROS2 publishers
  cmd_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planning/cmd_vis", 10);
  traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planning/travel_traj", 10);
  pos_cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>("position_cmd", 10);

  // ROS2 timer
  pos_cmd_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&BsplineTrackerNode::PosCmdTimerCallback, this));
  vis_cmd_timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&BsplineTrackerNode::VisCmdTimerCallback, this));
}

void BsplineTrackerNode::displayTrajWithColor(const std::vector<Eigen::Vector3d>& path, double resolution,
                        const Eigen::Vector4d& color, int id) {

  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = this->now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;
  traj_pub_->publish(mk);

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.w = 1.0;

  mk.color.r = static_cast<float>(color(0));
  mk.color.g = static_cast<float>(color(1));
  mk.color.b = static_cast<float>(color(2));
  mk.color.a = static_cast<float>(color(3));

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::msg::Point pt;
  for (auto& p : path) {
  pt.x = p(0);
  pt.y = p(1);
  pt.z = p(2);
  mk.points.push_back(pt);
  }
  traj_pub_->publish(mk);

}

void BsplineTrackerNode::drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
                 const Eigen::Vector4d& color) {
  visualization_msgs::msg::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = this->now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::msg::Marker::ARROW;
  mk_state.action = visualization_msgs::msg::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::msg::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = static_cast<float>(color(0));
  mk_state.color.g = static_cast<float>(color(1));
  mk_state.color.b = static_cast<float>(color(2));
  mk_state.color.a = static_cast<float>(color(3));

  cmd_vis_pub_->publish(mk_state);
}

void BsplineTrackerNode::bsplineCallback(const fast_planner_msgs::msg::Bspline::SharedPtr msg) {
  // parse pos traj

  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];
  }

  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}
void BsplineTrackerNode::replanCallback(const std_msgs::msg::Empty::SharedPtr msg) {
  // reset duration
  const double time_out = 0.01;
  auto time_now = this->now();
  double t_stop = (time_now - start_time_).seconds() + time_out;
  traj_duration_ = std::min(t_stop, traj_duration_);
}

void BsplineTrackerNode::newCallback(const std_msgs::msg::Empty::SharedPtr msg) {
  traj_cmd_.clear();
  traj_real_.clear();
}

void BsplineTrackerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (msg->child_frame_id == "X" || msg->child_frame_id == "O") return;

  traj_real_.push_back(Eigen::Vector3d(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z));

  if (traj_real_.size() > 10000) {
    traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
  }
}

void BsplineTrackerNode::VisCmdTimerCallback()
{
  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964, 1), 1);
  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

void BsplineTrackerNode::PosCmdTimerCallback()
{
  // No publishing before receiving a trajectory
  if (!receive_traj_) return;

  rclcpp::Time time_now = this->now();
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos, vel, acc, pos_f;
  double yaw, yawdot;

  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];

    double tf = std::min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);

  } else if (t_cur >= traj_duration_) {
    // Hover when finishing trajectory
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];

    pos_f = pos;

  } else {
    RCLCPP_WARN(this->get_logger(), "[Traj server]: invalid time.");
  }

  quadrotor_msgs::msg::PositionCommand cmd;
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  //cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  //cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;

  auto pos_err = pos_f - pos;
  last_yaw_ = cmd.yaw;

  pos_cmd_pub_->publish(cmd);

  // Draw arrow for yaw
  Eigen::Vector3d dir(std::cos(yaw), std::sin(yaw), 0.0);
  drawCmd(pos, 2.0 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));

  // Keep track of published positions
  traj_cmd_.push_back(pos);
  if (traj_cmd_.size() > 10000) {
    traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
  }
}

}  // namespace fast_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fast_planner::BsplineTrackerNode)
