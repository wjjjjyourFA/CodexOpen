#include "modules/common/math/pso_optimizer/pso_optimizer_simple.h"

PsoOptimizerSimple::PsoOptimizerSimple(int swarm_size, int dimensions,
                                       const Position& lower_bounds,
                                       const Position& upper_bounds,
                                       ObjectiveFunc objective_func)
    : swarm_size_(swarm_size),
      dimensions_(dimensions),
      lower_bounds_(lower_bounds),
      upper_bounds_(upper_bounds),
      objective_func_(objective_func),
      uniform_dist_(0.0, 1.0),
      rng_(std::random_device{}()) {}

void PsoOptimizerSimple::InitializeSwarm() {
  swarm_.resize(swarm_size_);
  for (auto& p : swarm_) {
    p.position.resize(dimensions_);
    p.velocity.resize(dimensions_);
    p.best_position.resize(dimensions_);

    // 随机初始化粒子的位置和速度
    // 位置在上下界之间分布
    for (int d = 0; d < dimensions_; ++d) {
      double min_val = lower_bounds_[d];
      double max_val = upper_bounds_[d];
      p.position[d]  = min_val + uniform_dist_(rng_) * (max_val - min_val);
      p.velocity[d]  = (uniform_dist_(rng_) - 0.5) * (max_val - min_val) * 0.1;
      p.best_position[d] = p.position[d];
    }

    // 计算粒子的初始最优值
    p.best_value = objective_func_(p.position);
    // 更新全局最优值 按最小化问题
    if (p.best_value < global_best_value_) {
      global_best_value_    = p.best_value;
      global_best_position_ = p.best_position;
    }
  }
}

void PsoOptimizerSimple::UpdateParticles() {
  for (auto& p : swarm_) {
    for (int d = 0; d < dimensions_; ++d) {
      double r1 = uniform_dist_(rng_);
      double r2 = uniform_dist_(rng_);

      p.velocity[d] =
          inertia_weight_ * p.velocity[d] +
          cognitive_coeff_ * r1 * (p.best_position[d] - p.position[d]) +
          social_coeff_ * r2 * (global_best_position_[d] - p.position[d]);

      p.position[d] += p.velocity[d];
      p.position[d] = clip(p.position[d], lower_bounds_[d], upper_bounds_[d]);
    }

    double value = objective_func_(p.position);
    if (value < p.best_value) {
      p.best_value    = value;
      p.best_position = p.position;
      if (value < global_best_value_) {
        global_best_value_    = value;
        global_best_position_ = p.position;
      }
    }
  }
}

void PsoOptimizerSimple::optimize(int max_iterations) {
  InitializeSwarm();
  for (int iter = 0; iter < max_iterations; ++iter) {
    UpdateParticles();
  }
}

Position PsoOptimizerSimple::GetBestPosition() const {
  return global_best_position_;
}

double PsoOptimizerSimple::GetBestValue() const { return global_best_value_; }

double PsoOptimizerSimple::clip(double value, double min, double max) {
  return std::max(min, std::min(value, max));
}
