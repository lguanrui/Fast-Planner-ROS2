/**
* This file is part of Fast-Planner-ROS2.
*
* Copyright 2024 Guanrui Li, Aerial-robot Control and Perception Lab, WPI
* Developed by Guanrui Li <lguanrui.github@gmail.com>
* for more information see <https://github.com/lguanrui/Fast-Planner-ROS2.git>.
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

#include <Eigen/Eigen>
#include <algorithm>
#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <fast_planner_msgs/msg/bspline.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
//#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

namespace fast_planner {

class KinoReplanFSM : public rclcpp::Node {
public:
  KinoReplanFSM(const rclcpp::NodeOptions &options)
      : Node("kino_replan_fsm", options),
        trigger_(false),
        have_target_(false),
        have_odom_(false),
        exec_state_(INIT),
        current_wp_(0) {
    init();
  }

  // Need this since we have Eigen members
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                              // target state
  int current_wp_;

  /* ROS2 utils */
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry> SyncPolicyImageOdom;
  //typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
  std::vector<std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>>>   sync_cam;
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> sync_subs_cam;
  std::vector<std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>>> sync_subs_odom;

  rclcpp::TimerBase::SharedPtr occ_timer_, esdf_timer_, vis_timer_;
  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr indep_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr update_range_pub_;
  rclcpp::Publisher<fast_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_, esdf_pub_, map_inf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_, depth_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;      // 0
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr topo_pub_;      // 1
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr predict_pub_;   // 2
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visib_pub_;     // 3, visibility constraints
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_pub_;  // 4, frontier searching
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr yaw_pub_;       // 5, yaw trajectory
  std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> vis_pubs_;  //

  //void publishESDF();
  //void publishUpdateRange();
  //void publishUnknown();
  //void publishDepth();

  /* helper functions */
  void init(); // initialize
  bool callKinodynamicReplan();        // front-end and back-end method
  bool callTopologicalTraj(int step);  // topo path guided gradient-based
                                       // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
  void printFSMExecState();
  void visGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                                              const Eigen::Vector4d& color, int id=0);
  //void visBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color, int id=0);
  void visBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                   int id2 = 0);
  void visGoal(const Eigen::Vector3d& goal, double resolution, const Eigen::Vector4d& color, int id=0);

  /* ROS2 functions */
  void execFSMCallback();
  void checkCollisionCallback();
  void waypointCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void depthOdomCallback(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
  void updateOccupancyCallback();
  void updateESDFCallback();
  void visCallback();

};

void KinoReplanFSM::init() {

  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  cout << "[KinoReplanFSM]: KinoReplanFSM init" << endl;

  /*  fsm param  */
  this->declare_parameter("fsm.flight_type", -1);
  this->declare_parameter("fsm.thresh_replan", -1.0);
  this->declare_parameter("fsm.thresh_no_replan", -1.0);
  this->declare_parameter("fsm.waypoint_num", -1);

  this->get_parameter("fsm.flight_type", target_type_);
  this->get_parameter("fsm.thresh_replan", replan_thresh_);
  this->get_parameter("fsm.thresh_no_replan", no_replan_thresh_);
  this->get_parameter("fsm.waypoint_num", waypoint_num_);

  for (int i = 0; i < waypoint_num_; i++) {
    this->declare_parameter("fsm.waypoint" + std::to_string(i) + "_x", -1.0);
    this->declare_parameter("fsm.waypoint" + std::to_string(i) + "_y", -1.0);
    this->declare_parameter("fsm.waypoint" + std::to_string(i) + "_z", -1.0);

    this->get_parameter("fsm.waypoint" + std::to_string(i) + "_x", waypoints_[i][0]);
    this->get_parameter("fsm.waypoint" + std::to_string(i) + "_y", waypoints_[i][1]);
    this->get_parameter("fsm.waypoint" + std::to_string(i) + "_z", waypoints_[i][2]);
  }

  RCLCPP_INFO(this->get_logger(), "fsm.flight_type: %d", target_type_);
  RCLCPP_INFO(this->get_logger(), "fsm.thresh_replan: %f", replan_thresh_);
  RCLCPP_INFO(this->get_logger(), "fsm.thresh_no_replan: %f", no_replan_thresh_);
  RCLCPP_INFO(this->get_logger(), "fsm.waypoint_num: %d", waypoint_num_);

  /* planner manager params */
  PlanParameters pp;
  this->declare_parameter("manager.max_vel", -1.0);
  this->declare_parameter("manager.max_acc", -1.0);
  this->declare_parameter("manager.max_jerk", -1.0);
  this->declare_parameter("manager.dynamic_environment", -1);
  this->declare_parameter("manager.clearance_threshold", -1.0);
  this->declare_parameter("manager.local_segment_length", -1.0);
  this->declare_parameter("manager.control_points_distance", -1.0);

  this->get_parameter("manager.max_vel", pp.max_vel_);
  this->get_parameter("manager.max_acc", pp.max_acc_);
  this->get_parameter("manager.max_jerk", pp.max_jerk_);
  this->get_parameter("manager.dynamic_environment", pp.dynamic_);
  this->get_parameter("manager.clearance_threshold", pp.clearance_);
  this->get_parameter("manager.local_segment_length", pp.local_traj_len_);
  this->get_parameter("manager.control_points_distance", pp.ctrl_pt_dist);

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;

  use_geometric_path = false;
  use_kinodynamic_path = true;
  use_topo_path = false;
  use_optimization = true;

  /* get sdf_map params*/
  MappingParameters mp;
  double x_size, y_size, z_size;
  this->declare_parameter("sdf_map.resolution", -1.0);
  this->declare_parameter("sdf_map.map_size_x", -1.0);
  this->declare_parameter("sdf_map.map_size_y", -1.0);
  this->declare_parameter("sdf_map.map_size_z", -1.0);
  this->declare_parameter("sdf_map.local_update_range_x", -1.0);
  this->declare_parameter("sdf_map.local_update_range_y", -1.0);
  this->declare_parameter("sdf_map.local_update_range_z", -1.0);
  this->declare_parameter("sdf_map.obstacles_inflation", -1.0);

  this->declare_parameter("sdf_map.fx", -1.0);
  this->declare_parameter("sdf_map.fy", -1.0);
  this->declare_parameter("sdf_map.cx", -1.0);
  this->declare_parameter("sdf_map.cy", -1.0);

  this->declare_parameter("sdf_map.use_depth_filter", true);
  this->declare_parameter("sdf_map.depth_filter_tolerance", 1.0);
  this->declare_parameter("sdf_map.depth_filter_maxdist", -1.0);
  this->declare_parameter("sdf_map.depth_filter_mindist", -1.0);
  this->declare_parameter("sdf_map.depth_filter_margin", -1);
  this->declare_parameter("sdf_map.k_depth_scaling_factor", -1.0);
  this->declare_parameter("sdf_map.skip_pixel", -1);

  this->declare_parameter("sdf_map.p_hit", 0.70);
  this->declare_parameter("sdf_map.p_miss", 0.35);
  this->declare_parameter("sdf_map.p_min", 0.12);
  this->declare_parameter("sdf_map.p_max", 0.97);
  this->declare_parameter("sdf_map.p_occ", 0.80);
  this->declare_parameter("sdf_map.min_ray_length", -0.1);
  this->declare_parameter("sdf_map.max_ray_length", -0.1);

  this->declare_parameter("sdf_map.esdf_slice_height", -0.1);
  this->declare_parameter("sdf_map.visualization_truncate_height", -0.1);
  this->declare_parameter("sdf_map.virtual_ceil_height", -0.1);

  this->declare_parameter("sdf_map.show_occ_time", false);
  this->declare_parameter("sdf_map.show_esdf_time", false);
  this->declare_parameter("sdf_map.pose_type", 1);

  this->declare_parameter("sdf_map.frame_id", std::string("world"));
  this->declare_parameter("sdf_map.local_bound_inflate", 1.0);
  this->declare_parameter("sdf_map.local_map_margin", 1);
  this->declare_parameter("sdf_map.ground_height", 1.0);

  this->get_parameter("sdf_map.resolution", mp.resolution_);
  this->get_parameter("sdf_map.map_size_x", mp.x_size_);
  this->get_parameter("sdf_map.map_size_y", mp.y_size_);
  this->get_parameter("sdf_map.map_size_z", mp.z_size_);
  this->get_parameter("sdf_map.local_update_range_x", mp.local_update_range_(0));
  this->get_parameter("sdf_map.local_update_range_y", mp.local_update_range_(1));
  this->get_parameter("sdf_map.local_update_range_z", mp.local_update_range_(2));
  this->get_parameter("sdf_map.obstacles_inflation", mp.obstacles_inflation_);

  this->get_parameter("sdf_map.fx", mp.fx_);
  this->get_parameter("sdf_map.fy", mp.fy_);
  this->get_parameter("sdf_map.cx", mp.cx_);
  this->get_parameter("sdf_map.cy", mp.cy_);

  this->get_parameter("sdf_map.use_depth_filter", mp.use_depth_filter_);
  this->get_parameter("sdf_map.depth_filter_tolerance", mp.depth_filter_tolerance_);
  this->get_parameter("sdf_map.depth_filter_maxdist", mp.depth_filter_maxdist_);
  this->get_parameter("sdf_map.depth_filter_mindist", mp.depth_filter_mindist_);
  this->get_parameter("sdf_map.depth_filter_margin", mp.depth_filter_margin_);
  this->get_parameter("sdf_map.k_depth_scaling_factor", mp.k_depth_scaling_factor_);
  this->get_parameter("sdf_map.skip_pixel", mp.skip_pixel_);

  this->get_parameter("sdf_map.p_hit", mp.p_hit_);
  this->get_parameter("sdf_map.p_miss", mp.p_miss_);
  this->get_parameter("sdf_map.p_min", mp.p_min_);
  this->get_parameter("sdf_map.p_max", mp.p_max_);
  this->get_parameter("sdf_map.p_occ", mp.p_occ_);
  this->get_parameter("sdf_map.min_ray_length", mp.min_ray_length_);
  this->get_parameter("sdf_map.max_ray_length", mp.max_ray_length_);

  this->get_parameter("sdf_map.esdf_slice_height", mp.esdf_slice_height_);
  this->get_parameter("sdf_map.visualization_truncate_height", mp.visualization_truncate_height_);
  this->get_parameter("sdf_map.virtual_ceil_height", mp.virtual_ceil_height_);

  this->get_parameter("sdf_map.show_occ_time", mp.show_occ_time_);
  this->get_parameter("sdf_map.show_esdf_time", mp.show_esdf_time_);
  this->get_parameter("sdf_map.pose_type", mp.pose_type_);

  this->get_parameter("sdf_map.frame_id", mp.frame_id_);
  this->get_parameter("sdf_map.local_bound_inflate", mp.local_bound_inflate_);
  this->get_parameter("sdf_map.local_map_margin", mp.local_map_margin_);
  this->get_parameter("sdf_map.ground_height", mp.ground_height_);

  /* kinodynamic astar params */
  KinodynamicAstarParams kap;
  this->declare_parameter("search.max_tau", -1.0);
  this->declare_parameter("search.init_max_tau", -1.0);
  this->declare_parameter("search.max_vel", -1.0);
  this->declare_parameter("search.max_acc", -1.0);
  this->declare_parameter("search.w_time", -1.0);
  this->declare_parameter("search.horizon", -1.0);
  this->declare_parameter("search.resolution_astar", -1.0);
  this->declare_parameter("search.time_resolution", -1.0);
  this->declare_parameter("search.lambda_heu", -1.0);
  this->declare_parameter("search.allocate_num", -1);
  this->declare_parameter("search.check_num", -1);
  this->declare_parameter("search.optimistic", true);
  this->declare_parameter("search.vel_margin", 0.0);

  this->get_parameter("search.max_tau", kap.max_tau);
  this->get_parameter("search.init_max_tau", kap.init_max_tau);
  this->get_parameter("search.max_vel", kap.max_vel);
  this->get_parameter("search.max_acc", kap.max_acc);
  this->get_parameter("search.w_time", kap.w_time);
  this->get_parameter("search.horizon", kap.horizon);
  this->get_parameter("search.resolution_astar", kap.resolution);
  this->get_parameter("search.time_resolution", kap.time_resolution);
  this->get_parameter("search.lambda_heu", kap.lambda_heu);
  this->get_parameter("search.allocate_num", kap.allocate_num);
  this->get_parameter("search.check_num", kap.check_num);
  this->get_parameter("search.optimistic", kap.optimistic);
  this->get_parameter("search.vel_margin", kap.vel_margin);

  /* optimization params */
  OptimizationParams opp;

  // Declare parameters
  this->declare_parameter<double>("optimization.lambda1", -1.0);
  this->declare_parameter<double>("optimization.lambda2", -1.0);
  this->declare_parameter<double>("optimization.lambda3", -1.0);
  this->declare_parameter<double>("optimization.lambda4", -1.0);
  this->declare_parameter<double>("optimization.lambda5", -1.0);
  this->declare_parameter<double>("optimization.lambda6", -1.0);
  this->declare_parameter<double>("optimization.lambda7", -1.0);
  this->declare_parameter<double>("optimization.lambda8", -1.0);

  this->declare_parameter<double>("optimization.dist0", -1.0);
  this->declare_parameter<double>("optimization.max_vel", -1.0);
  this->declare_parameter<double>("optimization.max_acc", -1.0);
  this->declare_parameter<double>("optimization.visib_min", -1.0);
  this->declare_parameter<double>("optimization.dlmin", -1.0);
  this->declare_parameter<double>("optimization.wnl", -1.0);

  this->declare_parameter<int>("optimization.max_iteration_num1", -1);
  this->declare_parameter<int>("optimization.max_iteration_num2", -1);
  this->declare_parameter<int>("optimization.max_iteration_num3", -1);
  this->declare_parameter<int>("optimization.max_iteration_num4", -1);

  this->declare_parameter<double>("optimization.max_iteration_time1", -1.0);
  this->declare_parameter<double>("optimization.max_iteration_time2", -1.0);
  this->declare_parameter<double>("optimization.max_iteration_time3", -1.0);
  this->declare_parameter<double>("optimization.max_iteration_time4", -1.0);

  this->declare_parameter<int>("optimization.algorithm1", -1);
  this->declare_parameter<int>("optimization.algorithm2", -1);
  this->declare_parameter<int>("optimization.order", -1);

  // Get parameters
  this->get_parameter("optimization.lambda1", opp.lambda1);
  this->get_parameter("optimization.lambda2", opp.lambda2);
  this->get_parameter("optimization.lambda3", opp.lambda3);
  this->get_parameter("optimization.lambda4", opp.lambda4);
  this->get_parameter("optimization.lambda5", opp.lambda5);
  this->get_parameter("optimization.lambda6", opp.lambda6);
  this->get_parameter("optimization.lambda7", opp.lambda7);
  this->get_parameter("optimization.lambda8", opp.lambda8);
  this->get_parameter("optimization.dist0", opp.dist0);
  this->get_parameter("optimization.max_vel", opp.max_vel);
  this->get_parameter("optimization.max_acc", opp.max_acc);
  this->get_parameter("optimization.visib_min", opp.visib_min);
  this->get_parameter("optimization.dlmin", opp.dlmin);
  this->get_parameter("optimization.wnl", opp.wnl);

  this->get_parameter("optimization.max_iteration_num1", opp.max_iteration_num[0]);
  this->get_parameter("optimization.max_iteration_num2", opp.max_iteration_num[1]);
  this->get_parameter("optimization.max_iteration_num3", opp.max_iteration_num[2]);
  this->get_parameter("optimization.max_iteration_num4", opp.max_iteration_num[3]);

  this->get_parameter("optimization.max_iteration_time1", opp.max_iteration_time[0]);
  this->get_parameter("optimization.max_iteration_time2", opp.max_iteration_time[1]);
  this->get_parameter("optimization.max_iteration_time3", opp.max_iteration_time[2]);
  this->get_parameter("optimization.max_iteration_time4", opp.max_iteration_time[3]);

  this->get_parameter("optimization.algorithm1", opp.algorithm1);
  this->get_parameter("optimization.algorithm2", opp.algorithm2);
  this->get_parameter("optimization.order", opp.order);

  // Create a timer to execute the FSM callback every 100 milliseconds
  exec_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&KinoReplanFSM::execFSMCallback, this));

  // Create a timer to check for collisions every 100 milliseconds
  safety_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&KinoReplanFSM::checkCollisionCallback, this));

  // Subscribe to the "waypoints" topic to receive waypoint messages
  waypoint_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "waypoints", 10, std::bind(&KinoReplanFSM::waypointCallback, this, std::placeholders::_1));

  // Subscribe to the "odom" topic to receive odometry messages
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&KinoReplanFSM::odometryCallback, this, std::placeholders::_1));

  // Create a publisher for the "replan" topic to signal replanning
  replan_pub_ = this->create_publisher<std_msgs::msg::Empty>("replan", 10);

 /* init callback */
  auto depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "depth");
  auto sync_odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "sync_depth_odom");
  auto depth_odom_sync = std::make_shared<message_filters::Synchronizer<SyncPolicyImageOdom>>(SyncPolicyImageOdom(10), *depth_sub_, *sync_odom_sub_);
  depth_odom_sync->registerCallback(std::bind(&KinoReplanFSM::depthOdomCallback, this, std::placeholders::_1, std::placeholders::_2));

  sync_cam.push_back(depth_odom_sync);
  sync_subs_cam.push_back(depth_sub_);
  sync_subs_odom.push_back(sync_odom_sub_);

// use odometry and point cloud
indep_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", 10, std::bind(&KinoReplanFSM::cloudCallback, this, std::placeholders::_1));

occ_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&KinoReplanFSM::updateOccupancyCallback, this));

esdf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&KinoReplanFSM::updateESDFCallback, this));

vis_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&KinoReplanFSM::visCallback, this));

map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/occupancy", 10);
map_inf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/occupancy_inflate", 10);
esdf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/esdf", 10);
update_range_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("sdf_map/update_range", 10);
unknown_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/unknown", 10);
depth_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/depth_cloud", 10);
bspline_pub_ = this->create_publisher<fast_planner_msgs::msg::Bspline>("planning/bspline", 10);

traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planning_vis/trajectory", 20);
vis_pubs_.push_back(traj_pub_);

topo_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planning_vis/topo_path", 20);
vis_pubs_.push_back(topo_pub_);

predict_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planning_vis/prediction", 20);
vis_pubs_.push_back(predict_pub_);

visib_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planning_vis/visib_constraint", 20);
vis_pubs_.push_back(visib_pub_);

frontier_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planning_vis/frontier", 20);
vis_pubs_.push_back(frontier_pub_);

yaw_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planning_vis/yaw", 20);
vis_pubs_.push_back(yaw_pub_);

  // Create a publisher for the "new" topic to signal new events

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(pp, mp, kap, opp, use_geometric_path, use_kinodynamic_path, use_topo_path,
                                    use_optimization);
  visualization_.reset(new PlanningVisualization());

}

void KinoReplanFSM::depthOdomCallback(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {

planner_manager_->sdf_map_->depthOdomCallback(depth_msg, odom_msg);

}

void KinoReplanFSM::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  planner_manager_->sdf_map_->cloudCallback(cloud_msg);
}

void KinoReplanFSM::updateOccupancyCallback() {
  planner_manager_->sdf_map_->updateOccupancyCallback();
}

void KinoReplanFSM::updateESDFCallback() {
  planner_manager_->sdf_map_->updateESDFCallback();
}

void KinoReplanFSM::visCallback() {
  // Placeholder for visualization callback
  sensor_msgs::msg::PointCloud2 map = planner_manager_->sdf_map_->publishMap();
  map_pub_->publish(map);

  sensor_msgs::msg::PointCloud2 map_inf = planner_manager_->sdf_map_->publishMapInflate(false);
  map_inf_pub_->publish(map_inf);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  std::cout << "Triggered!" << std::endl;
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  visGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}


void KinoReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  planner_manager_->sdf_map_->odomCallback(msg);

  have_odom_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" }; 
  cout << "[FSM]: I'm printing the FSM state" << endl;
  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback() {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    cout << "Executing FSM" << endl;
    if (!have_odom_) std::cout << "no odom." << std::endl;
    if (!trigger_) std::cout << "wait for goal." << std::endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = callKinodynamicReplan();
      if (success) {
        //cout << "[FSM]: the Kinodynamic replan is successful" << endl;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        //cout << "[FSM]: the state is now EXEC_TRAJ" << endl;
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajData* info     = &planner_manager_->local_data_;
      //cout << "[FSM]: I'm in the EXEC_TRAJ state" << endl;
      // ros::Time      time_now = ros::Time::now();
      rclcpp::Time curr_time = this->now();
      double         t_cur    = (curr_time - info->start_time_).seconds();
      t_cur                   = std::min(info->duration_, t_cur);
      //cout << "[FSM]: t_cur: " << t_cur << endl;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // std::cout << "near end" << std::endl;
        return;

      } else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
        // std::cout << "near start" << std::endl;
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajData* info     = &planner_manager_->local_data_;
      // ros::Time      time_now = ros::Time::now();
      //cout << "[FSM]: I'm in the REPLAN_TRAJ state" << endl;
      rclcpp::Time curr_time = this->now();
      double         t_cur    = (curr_time - info->start_time_).seconds();
      //cout << "[FSM]: REPLAN_TRAJ t_cur: " << t_cur << endl;

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      std_msgs::msg::Empty replan_msg;
      replan_pub_->publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback() {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        //visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
        visGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::msg::Empty emt;
        replan_pub_->publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist, this->now());

    if (!safe) {
      // cout << "current traj in collision." << endl;
      RCLCPP_WARN(this->get_logger(), "current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

void KinoReplanFSM::visGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                                              const Eigen::Vector4d& color, int id) {
  // Delete the previous path
  visualization_msgs::msg::Marker mk = visualization_->deleteSphereList(PATH + id % 100);
  mk.header.stamp = this->now(); 
  vis_pubs_[0]->publish(mk);

  // Draw the new path
  visualization_msgs::msg::Marker path_marker;
  path_marker = visualization_->displaySphereList(path, resolution, color, PATH + id % 100);
  path_marker.header.stamp = this->now();
  vis_pubs_[0]->publish(path_marker);
}

void KinoReplanFSM::visBspline(NonUniformBspline& bspline, double size,
                                        const Eigen::Vector4d& color, bool show_ctrl_pts, double size2,
                                        const Eigen::Vector4d& color2, int id1, int id2) {
  // If the bspline is empty, return
  if (bspline.getControlPointSize() == 0) return;

  // Evaluate the bspline
  vector<Eigen::Vector3d> traj_pts;
  double                  tm, tmp;
  bspline.getTimeSpan(tm, tmp);

  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }

  // Delete the previous bspline
  visualization_msgs::msg::Marker del_bsp_mk = visualization_->deleteSphereList(BSPLINE + id1 % 100);
  del_bsp_mk.header.stamp = this->now();
  vis_pubs_[0]->publish(del_bsp_mk);

  // Draw the new bspline
  visualization_msgs::msg::Marker bspline_marker;
  bspline_marker = visualization_->displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);
  bspline_marker.header.stamp = this->now();
  vis_pubs_[0]->publish(bspline_marker);

  // draw the control point
  if (!show_ctrl_pts) return;

  // Get the control points of the bspline
  Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();
  vector<Eigen::Vector3d> ctp;

  // Convert the control points to a vector of Eigen::Vector3d
  for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
    Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
    ctp.push_back(pt);
  }

  // Delete the previous control points visualization
  visualization_msgs::msg::Marker del_ctp_mk = visualization_->deleteSphereList(BSPLINE_CTRL_PT + id2 % 100);
  del_ctp_mk.header.stamp = this->now();
  vis_pubs_[0]->publish(del_ctp_mk);

  // Draw the new control points
  visualization_msgs::msg::Marker ctrl_pts_marker;
  ctrl_pts_marker = visualization_->displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
  ctrl_pts_marker.header.stamp = this->now();
  vis_pubs_[0]->publish(ctrl_pts_marker);

}

void KinoReplanFSM::visGoal(const Eigen::Vector3d& goal, double resolution, const Eigen::Vector4d& color, int id) {
  visualization_msgs::msg::Marker mk = visualization_->deleteSphereList(GOAL + id % 100);
  mk.header.stamp = this->now();
  vis_pubs_[0]->publish(mk);

  visualization_msgs::msg::Marker goal_marker;
  std::vector<Eigen::Vector3d> goal_vec = { goal };
  goal_marker = visualization_->displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
  goal_marker.header.stamp = this->now();
  vis_pubs_[0]->publish(goal_marker);
}

bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, this->now());

  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    fast_planner_msgs::msg::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_->publish(bspline);
    cout << "[FSM]: published the bspline." << endl;

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    //visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(0, 1, 0, 0.4), 0);
    visBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

}  // namespace fast_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fast_planner::KinoReplanFSM)