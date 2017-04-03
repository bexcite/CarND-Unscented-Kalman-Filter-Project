#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  size_t N = estimations.size();
  for (size_t i = 0; i < N; ++i) {
    VectorXd d = estimations[i] - ground_truth[i];
    VectorXd dd = d.array() * d.array();
    rmse = rmse + dd;
  }

  rmse = rmse/N;
  rmse = rmse.array().sqrt();

  return rmse;


}
