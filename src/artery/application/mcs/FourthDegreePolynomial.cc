#include <iostream>
#include "FourthDegreePolynomial.h"
#include <Eigen/Dense>
#include <cmath>

namespace artery {

FourthDegreePolynomial::FourthDegreePolynomial(double xInit,
                                               double vInit,
                                               double aInit,
                                               double vFinal,
                                               double aFinal,
                                               double duration)
{
    // 基本係数 (t = 0 の境界条件)
    a0 = xInit;
    a1 = vInit;
    a2 = aInit / 2.0; // x''(0) = 2 * a2 = aInit → a2 = aInit / 2

    // 残り 2 係数 (a3, a4) を線形方程式で解く
    const double T  = duration;
    const double T2 = T * T;
    const double T3 = T2 * T;

    Eigen::Matrix2d A;
    // [ 3*T^2  4*T^3 ] [a3] = [ vFinal - (a1 + 2*a2*T) ]
    // [ 6*T    12*T^2] [a4]   [ aFinal - (2*a2)        ]
    A << 3.0 * T2, 4.0 * T3,
         6.0 * T , 12.0 * T2;

    Eigen::Vector2d b;
    b << vFinal - (a1 + 2.0 * a2 * T),
         aFinal - (2.0 * a2);

    Eigen::Vector2d sol = A.colPivHouseholderQr().solve(b);
    a3 = sol(0);
    a4 = sol(1);
}

//---------------------------------------------------------------------
//   各階微分の計算
//---------------------------------------------------------------------

double FourthDegreePolynomial::calc_x(double t) const
{
    return a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t;
}

double FourthDegreePolynomial::calc_v(double t) const
{
    return a1 + 2.0 * a2 * t + 3.0 * a3 * t * t + 4.0 * a4 * t * t * t;
}

double FourthDegreePolynomial::calc_a(double t) const
{
    return 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t * t;
}

double FourthDegreePolynomial::calc_j(double t) const
{
    return 6.0 * a3 + 24.0 * a4 * t;
}

} // namespace artery

// ユニットテスト
// 実行方法
// mcsディレクトリシタで $g++ -DFOURTH_DEGREE_POLYNOMIAL_TEST FourthDegreePolynomial.cc
#ifdef FOURTH_DEGREE_POLYNOMIAL_TEST
int main() {
	// テスト例
	artery::FourthDegreePolynomial fdp(0.0, 0.0, 0.0, 2.0, 0.0, 2.0);

	double t = 1.0;
    std::cout << "Position at t = " << t << ": " << fdp.calc_x(t) << std::endl;
    std::cout << "Velocity at t = " << t << ": " << fdp.calc_v(t) << std::endl;
    std::cout << "Acceleration at t = " << t << ": " << fdp.calc_a(t) << std::endl;
    std::cout << "Jerk at t = " << t << ": " << fdp.calc_j(t) << std::endl;
	return 0;
}
#endif
