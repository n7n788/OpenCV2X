#include <iostream>
#include "QuarticPolynomial.h"

namespace artery
{

// コンストラクタ
QuarticPolynomial::QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double time) {
    a0 = xs;
    a1 = vxs;
    a2 = axs / 2.0;

    // 行列Aとベクトルbを定義
    Eigen::Matrix2d A;
    A << 3 * std::pow(time, 2), 4 * std::pow(time, 3),
         6 * time, 12 * std::pow(time, 2);

    Eigen::Vector2d b;
    b << vxe - a1 - 2 * a2 * time,
         axe - 2 * a2;

    // Ax = b を解く
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

    a3 = x[0];
    a4 = x[1];

    this->convergenceTime = time;
    this->targetSpeed = vxe;
}

// 任意の時刻tにおける位置を計算
double QuarticPolynomial::calc_point(double t) const {
    return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) + a4 * std::pow(t, 4);
}

// 任意の時刻tにおける速度を計算
double QuarticPolynomial::calc_first_derivative(double t) const {
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3);
}

// 任意の時刻tにおける加速度を計算
double QuarticPolynomial::calc_second_derivative(double t) const {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2);
}

// 任意の時刻tにおける躍度を計算
double QuarticPolynomial::calc_third_derivative(double t) const {
    return 6 * a3 + 24 * a4 * t;
}

}
// ユニットテスト
// 実行方法
// mcsディレクトリシタで $g++ -DQUARTIC_TEST QuarticPolynomial.cc
#ifdef QUARTIC_TEST
int main() {
	// テスト例
	artery::QuarticPolynomial qp(0.0, 0.0, 0.0, 2.0, 0.0, 2.0);

	double t = 1.0;
    std::cout << "Position at t = " << t << ": " << qp.calc_point(t) << std::endl;
    std::cout << "Velocity at t = " << t << ": " << qp.calc_first_derivative(t) << std::endl;
    std::cout << "Acceleration at t = " << t << ": " << qp.calc_second_derivative(t) << std::endl;
    std::cout << "Jerk at t = " << t << ": " << qp.calc_third_derivative(t) << std::endl;
    std::cout << "Time: " << qp.getConvergenceTime() << std::endl;
    std::cout << "Speed: " << qp.getTargetSpeed() << std::endl;
	return 0;
}
#endif
