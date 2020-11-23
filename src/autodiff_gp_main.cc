#include <stdio.h>

#include <vector>

#include "shared/math/math_util.h"
#include "glog/logging.h"
#include "eigen3/Eigen/Dense"

using math_util::Sq;

// Implement a Gaussian Process Regressor for N input dimensions, and M output
// dimensions, with the kernel type Kernel
// That is, the GP approximates f: R^N -> R^M
template <int N, int M, typename Kernel> class GaussianProcessRegression {
  typedef Eigen::Matrix<float, M> kOutputType;
  typedef Eigen::Matrix<float, N> kInputType;

  // Constructor to initialize the regressor with the provided input / output
  // pairs. 
  // inputs must be N x D, outputs M x D, for a total of D input / output pairs.
  explicit GaussianProcessRegression(
      const Eigen::MatrixXf<float>& inputs,
      const Eigen::MatrixXf<float>& outputs,
      Kernel* kernel),
      num_datapoints_(inputs.cols()) : 
        inputs_(inputs), 
        outputs_(outputs), 
        kernel_(kernel),
         {
    CHECK_EQ(inputs.rows(), N);
    CHECK_EQ(outputs.rows(), M);
    CHECK_EQ(inputs.cols(), outputs.cols());
    // Create the K matrix.
    // Invert it once for fast reuse later.
  }

  // Templated inference.
  template<typename T> 
  Eigen::Matrix<float, M, 1> Inference(const Eigen::Matrix<float, N, 1>& x) {
    Eigen::Matrix<float, M, 1> y = Eigen::Matrix<float, M, 1>::Zero();
    return y;
  }

  Eigen::MatrixXf<float> inputs_;
  Eigen::MatrixXf<float> outputs_;
  Kernel* kernel_;
  int num_datapoints_;
};

template <int N>
struct GaussianKernel {
  typedef Eigen::Matrix<float, N> kInputType;
  explicit GaussianKernel(float length) : length_(length) {}

  template<typename T> T operator(const kInputType& x1, const kInputType& x2) {
    return ((x1 - x2).squaredNorm() / Sq(static_cast<T>(length_)));
  }
  float length_;
};

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


int main(int argc, char* argv) {
  google::InitGoogleLogging(argv[0]);
  return 0;
}