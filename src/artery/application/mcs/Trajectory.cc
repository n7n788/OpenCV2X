#include "Trajectory.h"
#include "Polynomial.h"
#include "QuinticPolynomial.h"
#include "QuarticPolynomial.h"
#include <vector>
#include <iostream>

namespace artery
{

Trajectory::Trajectory(const artery::Polynomial& p) {
	// 位置を追加
	for (double t = 0.0; t < TIME_LENGTH; t += TIME_STEP) {
		mPoses.emplace_back(p.calc_point(t));
	}
	// 速度を追加
	for (double t = 0.0; t < TIME_LENGTH; t += TIME_STEP) {
		mSpeeds.emplace_back(p.calc_first_derivative(t));
	}
	// 加速度を追加
	for (double t = 0.0; t < TIME_LENGTH; t += TIME_STEP) {
		mAccels.emplace_back(p.calc_second_derivative(t));
	}
	// ジャーク追加
	for (double t = 0.0; t < TIME_LENGTH; t += TIME_STEP) {
		mJerks.emplace_back(p.calc_third_derivative(t));
	}
}

}

// ユニットテスト
// 実行方法
// mcsディレクトリ下で $g++ -DTRAJECTORY_TEST Trajectory.cc QuinticPolynomial.cc QuarticPolynomial.cc
#ifdef TRAJECTORY_TEST
int main() {
	artery::QuinticPolynomial qp1(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 5.0);
	artery::QuarticPolynomial qp2(0.0, 0.0, 0.0, 2.0, 0.0, 5.0);

	artery::Trajectory tj1(qp1);
	artery::Trajectory tj2(qp2);

	for (int i = 0; i < tj1.getPoses().size(); i++) {
		std::cout << "pos: " << tj1.getPoses().at(i) << " "
				  << "speed: " << tj1.getSpeeds().at(i) << " "
				  << "accel: " << tj1.getAccels().at(i) << " "
				  << "jerk: " << tj1.getJerks().at(i) << std::endl;
	}

	for (int i = 0; i < tj2.getPoses().size(); i++) {
		std::cout << "pos: " << tj2.getPoses().at(i) << " "
				  << "speed: " << tj2.getSpeeds().at(i) << " "
				  << "accel: " << tj2.getAccels().at(i) << " "
				  << "jerk: " << tj2.getJerks().at(i) << std::endl;
	}
}
#endif
