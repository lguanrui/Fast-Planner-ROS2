#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

// #include <path_searching/matrix_hash.h>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include "plan_env/edt_environment.h"
#include <path_searching/plan_utils.h>

namespace fast_planner {
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3

struct KinodynamicAstarParams {
  double max_tau, init_max_tau, max_vel, max_acc, w_time, horizon;
  double resolution, time_resolution;
  double lambda_heu;
  int allocate_num, check_num;
  bool optimistic;
  double vel_margin;
};

class KinodynamicAstar {
 private:
  /* ---------- main data structure ---------- */
  vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;
  std::vector<PathNodePtr> path_nodes_;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
  // shared_ptr<SDFMap> sdf_map;
  EDTEnvironment::Ptr edt_environment_;
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_;
  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  double tie_breaker_;
  bool optimistic_;

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  vector<double> cubic(double a, double b, double c, double d);
  vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double& optimal_time);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                    Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                    double tau);

 public:
  KinodynamicAstar(){};
  ~KinodynamicAstar();

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };

  /* main API */
  void setParam(KinodynamicAstarParams& kap);
  void init();
  void reset();
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
             Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0);

  void setEnvironment(const EDTEnvironment::Ptr& env);

  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  void getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                  vector<Eigen::Vector3d>& start_end_derivatives);

  std::vector<PathNodePtr> getVisitedNodes();

  typedef shared_ptr<KinodynamicAstar> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif