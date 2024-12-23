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



#ifndef _TOPO_PRM_H
#define _TOPO_PRM_H

#include <plan_env/edt_environment.h>
#include <plan_env/raycast.h>
#include <random>
#include <path_searching/plan_utils.h>

namespace fast_planner {

class TopologyPRM {
private:
  /* data */
  EDTEnvironment::Ptr edt_environment_;  // environment representation

  // sampling generator
  random_device rd_;
  default_random_engine eng_;
  uniform_real_distribution<double> rand_pos_;

  Eigen::Vector3d sample_r_;
  Eigen::Vector3d translation_;
  Eigen::Matrix3d rotation_;

  // roadmap data structure, 0:start, 1:goal, 2-n: others
  list<GraphNode::Ptr> graph_;
  vector<vector<Eigen::Vector3d>> raw_paths_;
  vector<vector<Eigen::Vector3d>> short_paths_;
  vector<vector<Eigen::Vector3d>> final_paths_;
  vector<Eigen::Vector3d> start_pts_, end_pts_;

  // raycasting
  vector<RayCaster> casters_;
  Eigen::Vector3d offset_;

  // parameter
  double max_sample_time_;
  int max_sample_num_;
  int max_raw_path_, max_raw_path2_;
  int short_cut_num_;
  Eigen::Vector3d sample_inflate_;
  double resolution_;

  double ratio_to_short_;
  int reserve_num_;

  bool parallel_shortcut_;

  /* create topological roadmap */
  /* path searching, shortening, pruning and merging */
  list<GraphNode::Ptr> createGraph(Eigen::Vector3d start, Eigen::Vector3d end);
  vector<vector<Eigen::Vector3d>> searchPaths();
  void shortcutPaths();
  vector<vector<Eigen::Vector3d>> pruneEquivalent(vector<vector<Eigen::Vector3d>>& paths);
  vector<vector<Eigen::Vector3d>> selectShortPaths(vector<vector<Eigen::Vector3d>>& paths, int step);

  /* ---------- helper ---------- */
  inline Eigen::Vector3d getSample();
  vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt);  // find pairs of visibile guard
  bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2,
                      Eigen::Vector3d pt);  // test redundancy with existing
                                            // connection between two guard
  bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh,
                 Eigen::Vector3d& pc, int caster_id = 0);
  bool triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2);
  void pruneGraph();

  void depthFirstSearch(vector<GraphNode::Ptr>& vis);

  vector<Eigen::Vector3d> discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2);
  vector<vector<Eigen::Vector3d>> discretizePaths(vector<vector<Eigen::Vector3d>>& path);

  vector<Eigen::Vector3d> discretizePath(vector<Eigen::Vector3d> path);
  void shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num = 1);

  vector<Eigen::Vector3d> discretizePath(const vector<Eigen::Vector3d>& path, int pt_num);
  bool sameTopoPath(const vector<Eigen::Vector3d>& path1, const vector<Eigen::Vector3d>& path2,
                    double thresh);
  Eigen::Vector3d getOrthoPoint(const vector<Eigen::Vector3d>& path);

  int shortestPath(vector<vector<Eigen::Vector3d>>& paths);

public:
  double clearance_;

  TopologyPRM(/* args */);
  ~TopologyPRM();

  void init(ros::NodeHandle& nh);

  void setEnvironment(const EDTEnvironment::Ptr& env);

  void findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end, vector<Eigen::Vector3d> start_pts,
                     vector<Eigen::Vector3d> end_pts, list<GraphNode::Ptr>& graph,
                     vector<vector<Eigen::Vector3d>>& raw_paths,
                     vector<vector<Eigen::Vector3d>>& filtered_paths,
                     vector<vector<Eigen::Vector3d>>& select_paths);

  double pathLength(const vector<Eigen::Vector3d>& path);
  vector<Eigen::Vector3d> pathToGuidePts(vector<Eigen::Vector3d>& path, int pt_num);

};

}  // namespace fast_planner

#endif