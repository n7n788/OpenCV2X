#include <iostream>
#include "QuinticPolynomial.h"

namespace artery
{

// Constructor
QuinticPolynomial::QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double time) {
    a0 = xs;
    a1 = vxs;
    a2 = axs / 2.0;

    // Matrix A and vector b
    Eigen::Matrix3d A;
    A << std::pow(time, 3), std::pow(time, 4), std::pow(time, 5),
         3 * std::pow(time, 2), 4 * std::pow(time, 3), 5 * std::pow(time, 4),
         6 * time, 12 * std::pow(time, 2), 20 * std::pow(time, 3);

    Eigen::Vector3d b;
    b << xe - a0 - a1 * time - a2 * std::pow(time, 2),
         vxe - a1 - 2 * a2 * time,
         axe - 2 * a2;

    // Solve Ax = b for x
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

    a3 = x[0];
    a4 = x[1];
    a5 = x[2];

    this->convergenceTime = time;
    this->targetSpeed = vxe;
}

// Calculate position at time t
double QuinticPolynomial::calc_point(double t) const {
    return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) + a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
}

// Calculate first derivative (velocity) at time t
double QuinticPolynomial::calc_first_derivative(double t) const {
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3) + 5 * a5 * std::pow(t, 4);
}

// Calculate second derivative (acceleration) at time t
double QuinticPolynomial::calc_second_derivative(double t) const {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2) + 20 * a5 * std::pow(t, 3);
}

// Calculate third derivative (jerk) at time t
double QuinticPolynomial::calc_third_derivative(double t) const {
    return 6 * a3 + 24 * a4 * t + 60 * a5 * std::pow(t, 2);
}

}

// Unit test or demonstration code (only compiled when this file is directly executed)
// 実行方法
// mcsディレクトリ下で $g++ -DQUINTIC_TEST QuinticPolynomial.cc
#ifdef QUINTIC_TEST
int main() {
    // Example usage for testing
    artery::QuinticPolynomial qp(0.0, 0.0, 0.0, 10.0, 2.0, 0.0, 2.0);

    double t = 1.0; // Time at which to evaluate
    std::cout << "Position at t = " << t << ": " << qp.calc_point(t) << std::endl;
    std::cout << "Velocity at t = " << t << ": " << qp.calc_first_derivative(t) << std::endl;
    std::cout << "Acceleration at t = " << t << ": " << qp.calc_second_derivative(t) << std::endl;
    std::cout << "Jerk at t = " << t << ": " << qp.calc_third_derivative(t) << std::endl;
    std::cout << "Time: " << qp.getConvergenceTime() << std::endl;
    std::cout << "Speed: " << qp.getTargetSpeed() << std::endl;
    return 0;
}
#endif

