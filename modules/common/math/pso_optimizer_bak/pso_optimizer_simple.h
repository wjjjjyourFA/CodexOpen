#ifndef PSO_OPTIMIZER_SIMPLE_H
#define PSO_OPTIMIZER_SIMPLE_H

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <functional>
#include <limits>
#include <random>

using Position = std::vector<double>;
using Velocity = std::vector<double>;

// 粒子结构体，包含位置、速度、最佳位置和最佳值
struct Particle {
  Position position;
  Velocity velocity;
  Position best_position;
  double best_value = std::numeric_limits<double>::infinity();
};

class PsoOptimizerSimple {
 public:
  // 目标函数，接受一个 Position，返回该位置的评价值（适应度）
  using ObjectiveFunc = std::function<double(const Position&)>;

  PsoOptimizerSimple(int swarm_size, int dimensions,
                     const Position& lower_bounds, const Position& upper_bounds,
                     ObjectiveFunc objective_func);

  // 执行粒子群优化的主入口，参数是最大迭代次数。
  void optimize(int max_iterations);
  Position GetBestPosition() const;
  double GetBestValue() const;

 private:
  int swarm_size_;  // 粒子群大小
  int dimensions_;  // 粒子维度
  // 粒子位置的上下界
  Position lower_bounds_;
  Position upper_bounds_;
  ObjectiveFunc objective_func_;
  int max_iter_ = 500;

  // 所有粒子对象的集合
  std::vector<Particle> swarm_;
  // 当前群体中最好的位置
  Position global_best_position_;
  double global_best_value_ = std::numeric_limits<double>::infinity();

  // 用于生成随机数的引擎和分布，供初始化和粒子更新时使用。 r1 r2
  std::default_random_engine rng_;
  std::uniform_real_distribution<double> uniform_dist_;

  // 用于生成随机数的引擎和分布，供初始化和粒子更新时使用。
  // PSO 的三个超参数：
  //   惯性权重 w
  //   个体学习因子 c1
  //   群体学习因子 c2
  double inertia_weight_  = 0.7;
  double cognitive_coeff_ = 1.5;
  double social_coeff_    = 1.5;

  // 初始化所有粒子的位置和速度，随机生成。
  void InitializeSwarm();
  // 迭代更新粒子的位置和速度
  void UpdateParticles();
  // 限制一个数值在 [min, max] 范围内，防止粒子越界。
  double clip(double value, double min, double max);
};

#endif