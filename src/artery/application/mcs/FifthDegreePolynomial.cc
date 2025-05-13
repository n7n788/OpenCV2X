#include <iostream>

#include "FifthDegreePolynomial.h"

#include <Eigen/Dense>
#include <cmath>

namespace artery {

FifthDegreePolynomial::FifthDegreePolynomial(double xInit,
                                             double vInit,
                                             double aInit,
                                             double xFinal,
                                             double vFinal,
                                             double aFinal,
                                             double duration)
{
    // t = 0 の境界条件から決定できる 0〜2 次係数
    a0 = xInit;
    a1 = vInit;
    a2 = aInit / 2.0;          // x''(0) = 2 a2 = aInit

    // 係数 a3, a4, a5 を線形方程式で求める
    const double T  = duration;
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    Eigen::Matrix3d A;
    A <<    T3,     T4,     T5,
           3*T2,   4*T3,   5*T4,
           6*T ,  12*T2,  20*T3;

    Eigen::Vector3d b;
    b << xFinal - (a0 + a1*T + a2*T2),
         vFinal - (a1 + 2*a2*T),
         aFinal - (2*a2);

    Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
    a3 = sol(0);
    a4 = sol(1);
    a5 = sol(2);
}

//---------------------------------------------------------------------
//  各階微分値の計算
//---------------------------------------------------------------------
double FifthDegreePolynomial::calc_x(double t) const
{
    return a0 + a1*t + a2*t*t + a3*t*t*t
         +  a4*t*t*t*t + a5*t*t*t*t*t;
}

double FifthDegreePolynomial::calc_v(double t) const
{
    return a1 + 2*a2*t + 3*a3*t*t
         + 4*a4*t*t*t + 5*a5*t*t*t*t;
}

double FifthDegreePolynomial::calc_a(double t) const
{
    return 2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;
}

double FifthDegreePolynomial::calc_j(double t) const
{
    return 6*a3 + 24*a4*t + 60*a5*t*t;
}

} // namespace artery


// ユニットテスト　実行方法
// mcsディレクトリ下で $g++ -DFIFTH_DEGREE_POLYNOMIAL_TEST FifthDegreePolynomial.cc
#ifdef FIFTH_DEGREE_POLYNOMIAL_TEST
int main() {
    // Example usage for testing
    artery::FifthDegreePolynomial fdp(0.0, 0.0, 0.0, 10.0, 2.0, 0.0, 2.0);

    double t = 1.0; // Time at which to evaluate
    std::cout << "Position at t = " << t << ": " << fdp.calc_x(t) << std::endl;
    std::cout << "Velocity at t = " << t << ": " << fdp.calc_v(t) << std::endl;
    std::cout << "Acceleration at t = " << t << ": " << fdp.calc_a(t) << std::endl;
    std::cout << "Jerk at t = " << t << ": " << fdp.calc_j(t) << std::endl;
    return 0;
}
#endif

