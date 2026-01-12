#include <iostream>

#include "pso_optimizer_simple.h"

// 示例目标函数：Sphere 函数 f(x) = Σx_i^2，最小值为 0
double SphereFunction(const std::vector<double>& x) {
  double sum = 0.0;
  for (double xi : x) sum += xi * xi;
  return sum;
}

int main() {
  int dimensions     = 2;
  int swarm_size     = 30;
  int max_iterations = 100;

  std::vector<double> lower_bounds(dimensions, -10.0);
  std::vector<double> upper_bounds(dimensions, 10.0);

  PsoOptimizerSimple optimizer(swarm_size, dimensions, lower_bounds,
                               upper_bounds, SphereFunction);
  optimizer.optimize(max_iterations);

  auto best_pos   = optimizer.GetBestPosition();
  double best_val = optimizer.GetBestValue();

  std::cout << "Best position: ";
  for (double x : best_pos) std::cout << x << " ";
  std::cout << "\nBest value: " << best_val << std::endl;

  return 0;
}
