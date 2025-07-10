
#include "pgo.hpp"    // 假设你把上面的头文件存为 pgo.hpp
#include <iostream>

int main() {

  PoseGraphOptimizer pgo;

  pgo.addKeyframe(0, Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,0,0));
  pgo.addKeyframe(1, Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,0,0));

  Eigen::Quaterniond q01(1,0,0,0);
  Eigen::Vector3d    t01(1.0, 0.0, 0.0);
  pgo.addConstraint(0, 1, q01, t01);

  std::cout << "Before optimization:\n";
  for (auto const& kv : pgo.optimizedKeyframes()) {
    std::cout << "  ID="<<kv.first
              << " t="<<kv.second.t.transpose()
              << " q="<<kv.second.q.coeffs().transpose()
              << "\n";
  }

  pgo.optimize(50);

  std::cout << "After optimization:\n";
  for (auto const& kv : pgo.optimizedKeyframes()) {
    std::cout << "  ID="<<kv.first
              << " t="<<kv.second.t.transpose()
              << " q="<<kv.second.q.coeffs().transpose()
              << "\n";
  }

  return 0;
}
