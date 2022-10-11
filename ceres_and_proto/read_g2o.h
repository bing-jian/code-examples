
#ifndef EXAMPLES_CERES_READ_G2O_H_
#define EXAMPLES_CERES_READ_G2O_H_

#include <fstream>
#include <string>

#include "glog/logging.h"
#include "core/proto_utils.h"
#include "proto/graph.pb.h"
#include "types.h"

namespace ceres::examples {

// Reads a single pose from the input and inserts it into the map. Returns false
// if there is a duplicate entry.
template <typename Pose, typename Allocator>
bool ReadVertex(std::ifstream* infile,
                std::map<std::string, Pose, std::less<std::string>, Allocator>* poses) {
    std::string id;
  Pose pose;
  *infile >> id >> pose;

  // Ensure we don't have duplicate poses.
  if (poses->find(id) != poses->end()) {
    LOG(ERROR) << "Duplicate vertex with ID: " << id;
    return false;
  }
  (*poses)[id] = pose;

  return true;
}

// Reads the constraints between two vertices in the pose graph
template <typename Constraint, typename Allocator>
void ReadConstraint(std::ifstream* infile,
                    std::vector<Constraint, Allocator>* constraints) {
  Constraint constraint;
  *infile >> constraint;

  constraints->push_back(constraint);
}

// Reads a file in the g2o filename format that describes a pose graph
// problem. The g2o format consists of two entries, vertices and constraints.
//
// In 2D, a vertex is defined as follows:
//
// VERTEX_SE2 ID x_meters y_meters yaw_radians
//
// A constraint is defined as follows:
//
// EDGE_SE2 ID_A ID_B A_x_B A_y_B A_yaw_B I_11 I_12 I_13 I_22 I_23 I_33
//
// where I_ij is the (i, j)-th entry of the information matrix for the
// measurement.
//
//
// In 3D, a vertex is defined as follows:
//
// VERTEX_SE3:QUAT ID x y z q_x q_y q_z q_w
//
// where the quaternion is in Hamilton form.
// A constraint is defined as follows:
//
// EDGE_SE3:QUAT ID_a ID_b x_ab y_ab z_ab q_x_ab q_y_ab q_z_ab q_w_ab I_11 I_12 I_13 ... I_16 I_22 I_23 ... I_26 ... I_66 // NOLINT
//
// where I_ij is the (i, j)-th entry of the information matrix for the
// measurement. Only the upper-triangular part is stored. The measurement order
// is the delta position followed by the delta orientation.
template <typename Pose,
          typename Constraint,
          typename MapAllocator,
          typename VectorAllocator>
bool ReadG2oFile(const std::string& filename,
                 std::map<std::string, Pose, std::less<std::string>, MapAllocator>* poses,
                 std::vector<Constraint, VectorAllocator>* constraints) {
  CHECK(poses != nullptr);
  CHECK(constraints != nullptr);

  poses->clear();
  constraints->clear();

  std::ifstream infile(filename.c_str());
  if (!infile) {
    return false;
  }

  std::string data_type;
  while (infile.good()) {
    // Read whether the type is a node or a constraint.
    infile >> data_type;
    if (data_type == Pose::name()) {
      if (!ReadVertex(&infile, poses)) {
        return false;
      }
    } else if (data_type == Constraint::name()) {
      ReadConstraint(&infile, constraints);
    } else {
      LOG(ERROR) << "Unknown data type: " << data_type;
      return false;
    }

    // Clear any trailing whitespace from the line.
    infile >> std::ws;
  }

  return true;
}

template <typename Pose,
          typename Constraint,
          typename MapAllocator,
          typename VectorAllocator>
bool ReadGraphFile(const std::string& graph_fname,
                 std::map<std::string, Pose, std::less<int>, MapAllocator>* poses,
                 std::vector<Constraint, VectorAllocator>* constraints) {
  CHECK(poses != nullptr);
  CHECK(constraints != nullptr);

  poses->clear();
  constraints->clear();

  bing::PanoGraph graph;
  bool pb_loaded = utils::LoadProtoFromFile(graph_fname, graph);
  if (!pb_loaded) {
      std::cerr << "Failed to load graph from " << graph_fname << std::endl;
      return false;
  }

  for (const auto& node : graph.nodes()) {
    (*poses)[node.id()].p(0) = node.pose().translation().x();
    (*poses)[node.id()].p(1) = node.pose().translation().y();
    (*poses)[node.id()].p(2) = node.pose().translation().z();
  }

  return true;
}

}  // namespace ceres::examples

#endif  // EXAMPLES_CERES_READ_G2O_H_
