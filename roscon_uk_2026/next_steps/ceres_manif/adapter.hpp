#pragma once
namespace calibration {
struct Report {
  int termination;
  int iterations;
  bool usable;
  bool converged;
  double initial_cost;
  double final_cost;
  double solve_seconds;
};
Report solve(const double* source, const double* target, int count, double* x,
             double huber_scale_m, int max_iterations, double tolerance);
void evaluate(const double* source, const double* target, const double* x,
              double* residual, double* jacobian);
void group_action(const double* x, const double* point, const double* delta,
                  double* output, double* right_jacobian);
void toolchain(int* output);
}
