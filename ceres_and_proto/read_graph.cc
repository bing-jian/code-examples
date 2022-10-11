#include <gflags/gflags.h>

#include <string>
#include "glog/logging.h"

#include "core/proto_utils.h"
#include "proto/graph.pb.h"

DEFINE_string(input_graph, "", "graph file in .pb or .pb.txt");

namespace {
static bool IsNonEmptyMessage(const char* flagname, const std::string& value) {
  return value[0] != '\0';
}
}  // namespace

DEFINE_validator(input_graph, &IsNonEmptyMessage);

namespace {
// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool OutputPoses(const std::string& filename, const bing::PanoGraph& graph) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  for (const auto& node : graph.nodes()) {
    outfile << node.id() << " "
            << node.pose().translation().x() <<  " "
            << node.pose().translation().y() <<  " "
            << node.pose().translation().z() <<  " "
            << 0 << " " << 0 << " "
            << 0 << " " << 1 << '\n';
  }
  return true;
}
}  // namespace

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("example program");
  gflags::ParseCommandLineFlags(&argc, &argv, true);


  std::string graph_fname(FLAGS_input_graph);

  bing::PanoGraph graph;
  bool pb_loaded = utils::LoadProtoFromFile(graph_fname, graph);
  if (!pb_loaded) {
      std::cerr << "Failed to load graph from " << graph_fname << std::endl;
      return 1;
  }
  OutputPoses("/tmp/poses.txt", graph);
  utils::SaveProtoToBinFile("/tmp/g.pb", graph);

  gflags::ShutDownCommandLineFlags();
  return 0;
}
