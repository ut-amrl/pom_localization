#include <stdio.h>

#include <vector>

#include "shared/math/math_util.h"
#include "glog/logging.h"
#include "eigen3/Eigen/Dense"

#include "gaussian_process/gp_regression.h"
#include "gaussian_process/kernel/periodic_gaussian_kernel.h"
#include "gaussian_process/kernel/pose_2d_kernel.h"

#include "pose_optimization/movable_observation_gp_cost_functor.h"

using math_util::Sq;
using gp_regression::GaussianProcessRegression;
using namespace gp_kernel;



// An observation residual. At construction, it takes in a GP regressor (the
// map), and an actual observation.
// When called, it takes in an input var (the robot's location), produces a 
// predicted output var, and the residual is the difference between the
// predicted and the actual observation.
struct GPObservationResidual {
  GPObservationResidual(
      GaussianProcessRegression<2, 2, GaussianKernel<2>>* gp,
      Eigen::Matrix<float, 2, 1>& observation) : 
          gp_(gp), observation_(observation) {}
  
  template<typename T> bool operator()(T* residuals, T* robot_pose_ptr) {
    Eigen::Matrix<T, 2, 1> x;
    // Convention: robot poses are in the order x, y, theta
    x(0) = robot_pose_ptr[0];
    x(1) = robot_pose_ptr[1];
    x = x + Eigen::Rotation2D<T>(robot_pose_ptr[2]) * observation_.cast<T>();

    Eigen::Matrix<T, 2, 1> y_predicted = gp_->Inference<T>(x);
    return true;
  }

  GaussianProcessRegression<2, 2, GaussianKernel<2>>* gp_;
  Eigen::Matrix<float, 2, 1>& observation_;
};



int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  return 0;
}