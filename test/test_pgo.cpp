#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

#include "pgo.hpp"  // 包含 PoseGraphOptimizer 定义

struct EdgeData {
  int id_a, id_b;
  Eigen::Quaterniond q_ab;
  Eigen::Vector3d    t_ab;
  std::vector<double> info;  // 6×6 信息矩阵（21 个上三角元素）
};

int main(int argc, char** argv) {

  std::string infile  = "/home/syx/wfzf/code/project/HBA_ws/src/HBA/data/pgo/input.g2o";
  std::string outfile = "/home/syx/wfzf/code/project/HBA_ws/src/HBA/data/pgo/output.g2o";

  std::map<int, Keyframe>   vertices;
  std::vector<EdgeData>     edges;

  std::ifstream fin(infile);
  if (!fin) {
    std::cerr << "Cannot open file: " << infile << "\n";
    return 1;
  }
  std::string tag;
  while (fin >> tag) {
    if (tag == "VERTEX_SE3:QUAT") {
      int id;
      double x,y,z,qx,qy,qz,qw;
      fin >> id >> x >> y >> z >> qx >> qy >> qz >> qw;
      Keyframe kf;
      kf.id = id;
      kf.t  = Eigen::Vector3d(x,y,z);
      kf.q  = Eigen::Quaterniond(qw, qx, qy, qz);
      vertices[id] = kf;
    }
    else if (tag == "EDGE_SE3:QUAT") {
      EdgeData e;
      double x,y,z,qx,qy,qz,qw;
      fin >> e.id_a >> e.id_b >> x >> y >> z >> qx >> qy >> qz >> qw;
      e.t_ab = Eigen::Vector3d(x,y,z);
      e.q_ab = Eigen::Quaterniond(qw, qx, qy, qz);
      e.info.resize(21);
      for (int i = 0; i < 21; ++i) fin >> e.info[i];
      edges.push_back(e);
    }
    else {
      std::string line;
      std::getline(fin, line);
    }
  }
  fin.close();

  PoseGraphOptimizer pgo;
  for (auto& kv : vertices) {
    pgo.addKeyframe(kv.first, kv.second.q, kv.second.t);
  }
  for (auto& e : edges) {
    pgo.addConstraint(e.id_a, e.id_b, e.q_ab, e.t_ab);
  }

  // std::cout << "Before optimization:\n";
  // for (auto const& kv : pgo.optimizedKeyframes()) {
  //   std::cout << "  ID="<<kv.first
  //             << " t="<<kv.second.t.transpose()
  //             << " q="<<kv.second.q.coeffs().transpose()
  //             << "\n";
  // }

  pgo.optimize(200);

  std::ofstream fout(outfile);
  if (!fout) {
    std::cerr << "Cannot open file for write: " << outfile << "\n";
    return 1;
  }

  for (auto const& kv : pgo.optimizedKeyframes()) {
    int id = kv.first;
    auto const& kf = kv.second;
    fout << "VERTEX_SE3:QUAT " << id << " "
         << kf.t.x() << " " << kf.t.y() << " " << kf.t.z() << " "
         << kf.q.x() << " " << kf.q.y() << " "
         << kf.q.z() << " " << kf.q.w() << "\n";
  }

  for (auto const& e : edges) {
    fout << "EDGE_SE3:QUAT "
         << e.id_a << " " << e.id_b << " "
         << e.t_ab.x() << " " << e.t_ab.y() << " " << e.t_ab.z() << " "
         << e.q_ab.x() << " " << e.q_ab.y() << " "
         << e.q_ab.z() << " " << e.q_ab.w();
    for (double v : e.info) {
      fout << " " << v;
    }
    fout << "\n";
  }
  fout.close();

  // std::cout << "After optimization:\n";
  // for (auto const& kv : pgo.optimizedKeyframes()) {
  //   std::cout << "  ID="<<kv.first
  //             << " t="<<kv.second.t.transpose()
  //             << " q="<<kv.second.q.coeffs().transpose()
  //             << "\n";
  // }

  return 0;
}


// int main() {

//   PoseGraphOptimizer pgo;

//   pgo.addKeyframe(0, Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,0,0));
//   pgo.addKeyframe(1, Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,0,0));

//   Eigen::Quaterniond q01(1,0,0,0);
//   Eigen::Vector3d    t01(1.0, 0.0, 0.0);
//   pgo.addConstraint(0, 1, q01, t01);

//   std::cout << "Before optimization:\n";
//   for (auto const& kv : pgo.optimizedKeyframes()) {
//     std::cout << "  ID="<<kv.first
//               << " t="<<kv.second.t.transpose()
//               << " q="<<kv.second.q.coeffs().transpose()
//               << "\n";
//   }

//   pgo.optimize(50);

//   std::cout << "After optimization:\n";
//   for (auto const& kv : pgo.optimizedKeyframes()) {
//     std::cout << "  ID="<<kv.first
//               << " t="<<kv.second.t.transpose()
//               << " q="<<kv.second.q.coeffs().transpose()
//               << "\n";
//   }

//   return 0;
// }
