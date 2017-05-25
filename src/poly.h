//
// Created by Alexey Simonov on 25/05/2017.
//

#ifndef MPC_POLY_H
#define MPC_POLY_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);


#endif //MPC_POLY_H
