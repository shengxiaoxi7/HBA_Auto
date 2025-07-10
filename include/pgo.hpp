// pgo.hpp

#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <map>
#include <vector>
#include <iostream>

/// 单点帧（关键帧）数据结构：包含 id、平移、四元数
struct Keyframe {
  int                  id;
  Eigen::Vector3d      t;
  Eigen::Quaterniond   q;
};

/// 约束（可用于连通帧或回环）：从 id_a 到 id_b 的相对测量 q_ab, t_ab
struct Constraint {
  int                   id_a, id_b;
  Eigen::Quaterniond    q_ab;
  Eigen::Vector3d       t_ab;
};

/// Pose Graph 优化器头文件
class PoseGraphOptimizer {
public:
  PoseGraphOptimizer() = default;
  ~PoseGraphOptimizer() = default;

  /// 添加一个关键帧
  void addKeyframe(int id, const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    Keyframe kf; kf.id = id; kf.q = q; kf.t = t;
    keyframes_[id] = kf;
  }

  /// 添加一条位姿约束（邻接或回环）
  void addConstraint(int id_a, int id_b, const Eigen::Quaterniond& q_ab, const Eigen::Vector3d& t_ab) {
    constraints_.push_back({id_a, id_b, q_ab, t_ab});
  }

  /// 执行优化（默认迭代 200 步）
  void optimize(int max_iterations = 200) {
    ceres::Problem problem;

    // 参数块：为每个关键帧的 q,t 添加
    for (auto& kv : keyframes_) {
      auto& kf = kv.second;
      problem.AddParameterBlock(kf.q.coeffs().data(), 4, new ceres::EigenQuaternionParameterization());
      problem.AddParameterBlock(kf.t.data(), 3, new ceres::IdentityParameterization(3));
    }

    // 固定第一个帧
    if (!keyframes_.empty()) {
      auto it = keyframes_.begin();
      problem.SetParameterBlockConstant(it->second.q.coeffs().data());
      problem.SetParameterBlockConstant(it->second.t.data());
    }

    // 添加 residual blocks
    for (auto& c : constraints_) {
      ceres::CostFunction* cost = PoseGraphErrorTerm::Create(c.q_ab, c.t_ab);
      problem.AddResidualBlock(cost, nullptr,
        keyframes_[c.id_a].q.coeffs().data(),
        keyframes_[c.id_a].t.data(),
        keyframes_[c.id_b].q.coeffs().data(),
        keyframes_[c.id_b].t.data());
    }

    // Solver 选项
    ceres::Solver::Options opts;
    opts.max_num_iterations           = max_iterations;
    opts.linear_solver_type           = ceres::SPARSE_NORMAL_CHOLESKY;
    opts.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
  }

  /// 获取优化后结果
  const std::map<int, Keyframe>& optimizedKeyframes() const {
    return keyframes_;
  }

private:
  std::map<int, Keyframe>   keyframes_;
  std::vector<Constraint>   constraints_;

  /// 内部: Ceres residual term
  struct PoseGraphErrorTerm {
    PoseGraphErrorTerm(const Eigen::Quaterniond& q_ab,
                       const Eigen::Vector3d& t_ab)
      : q_ab_(q_ab), t_ab_(t_ab) {}

    template <typename T>
    bool operator()(const T* const q_a, const T* const t_a,
                    const T* const q_b, const T* const t_b,
                    T* residuals) const {
      Eigen::Map<const Eigen::Quaternion<T>> QA(q_a);
      Eigen::Map<const Eigen::Matrix<T,3,1>> TA(t_a);
      Eigen::Map<const Eigen::Quaternion<T>> QB(q_b);
      Eigen::Map<const Eigen::Matrix<T,3,1>> TB(t_b);

      Eigen::Quaternion<T> Qab_meas = q_ab_.template cast<T>();
      Eigen::Matrix<T,3,1> Tab_meas = t_ab_.template cast<T>();

      // 旋转误差
      Eigen::Quaternion<T> Qerr = Qab_meas.inverse() * (QA.inverse() * QB);
      // 平移误差
      Eigen::Matrix<T,3,1> Terr = QA.inverse() * (TB - TA) - Tab_meas;
      Sophus::SO3<T> Rerr(Qerr);
      Eigen::Matrix<T,3,1> omega = Rerr.log(); 

      // 填 residuals
      residuals[0] = omega[0];
      residuals[1] = omega[1];
      residuals[2] = omega[2];
      residuals[3] = Terr.x();
      residuals[4] = Terr.y();
      residuals[5] = Terr.z();
      return true;
    }

    static ceres::CostFunction* Create(const Eigen::Quaterniond& q_ab,
                                       const Eigen::Vector3d& t_ab) {
      return new ceres::AutoDiffCostFunction<
          PoseGraphErrorTerm, 6, 4, 3, 4, 3>(
            new PoseGraphErrorTerm(q_ab, t_ab));
    }

    Eigen::Quaterniond q_ab_;
    Eigen::Vector3d    t_ab_;
  };
};


// int main(int argc, char** argv) {
//   // 1. 构造优化器
//   PoseGraphOptimizer pgo;

//   // 2. 添加关键帧
//   for (auto& kf : keyframes_vector) {
//     pgo.addKeyframe(kf.id, kf.q, kf.t);
//   }

//   // 3. 添加约束（Odometry + Loop）
//   for (auto& c : constraints_vector) {
//     pgo.addConstraint(c.id_a, c.id_b, c.q_ab, c.t_ab);
//   }

//   // 4. 运行优化
//   pgo.optimize(200);

//   // 5. 获取并输出结果
//   for (auto const& kv : pgo.optimizedKeyframes()) {
//     std::cout << "ID=" << kv.first
//               << " q=" << kv.second.q.coeffs().transpose()
//               << " t=" << kv.second.t.transpose()
//               << std::endl;
//   }
//   return 0;
// }