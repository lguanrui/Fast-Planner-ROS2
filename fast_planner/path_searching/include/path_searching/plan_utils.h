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



#ifndef _PLAN_UTILS_H
#define _PLAN_UTILS_H

namespace fast_planner {

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time;  // dyn
  int time_idx;
  PathNode* parent;
  char node_state;

  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;

class NodeComparator {
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};


/* ---------- used for iterating all topo combination ---------- */
class TopoIterator {
private:
  /* data */
  vector<int> path_nums_;
  vector<int> cur_index_;
  int combine_num_;
  int cur_num_;

  void increase(int bit_num) {
    cur_index_[bit_num] += 1;
    if (cur_index_[bit_num] >= path_nums_[bit_num]) {
      cur_index_[bit_num] = 0;
      increase(bit_num + 1);
    }
  }

public:
  TopoIterator(vector<int> pn) {
    path_nums_ = pn;
    cur_index_.resize(path_nums_.size());
    fill(cur_index_.begin(), cur_index_.end(), 0);
    cur_num_ = 0;

    combine_num_ = 1;
    for (int i = 0; i < path_nums_.size(); ++i) {
      combine_num_ *= path_nums_[i] > 0 ? path_nums_[i] : 1;
    }
    std::cout << "[Topo]: merged path num: " << combine_num_ << std::endl;
  }
  TopoIterator() {
  }
  ~TopoIterator() {
  }

  bool nextIndex(vector<int>& index) {
    index = cur_index_;
    cur_num_ += 1;

    if (cur_num_ == combine_num_) return false;

    // go to next combination
    increase(0);
    return true;
  }
};

/* ---------- node of topo graph ---------- */
class GraphNode {
private:
  /* data */

public:
  enum NODE_TYPE { Guard = 1, Connector = 2 };

  enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3 };

  GraphNode(/* args */) {
  }
  GraphNode(Eigen::Vector3d pos, NODE_TYPE type, int id) {
    pos_ = pos;
    type_ = type;
    state_ = NEW;
    id_ = id;
  }
  ~GraphNode() {
  }

  vector<shared_ptr<GraphNode>> neighbors_;
  Eigen::Vector3d pos_;
  NODE_TYPE type_;
  NODE_STATE state_;
  int id_;

  typedef shared_ptr<GraphNode> Ptr;
};

}

#endif