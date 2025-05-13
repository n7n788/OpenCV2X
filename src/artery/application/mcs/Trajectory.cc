#include "Trajectory.h"
#include "Polynomial.h"
#include <vector>

namespace artery
{

Trajectory::Trajectory(const artery::Polynomial& p, double duration) {
	mDuration = duration;
	// 位置を追加
	for (double t = 0.0; t < mDuration; t += TIME_STEP) {
		mPoses.emplace_back(p.calc_x(t));
		mSpeeds.emplace_back(p.calc_v(t));
		mAccels.emplace_back(p.calc_a(t));
		mJerks.emplace_back(p.calc_j(t));
	}
}

}

// ユニットテスト
// 実行方法
// mcsディレクトリ下で $g++ -DTRAJECTORY_TEST Trajectory.cc FourthDegreePolynomial.cc FifthDegreePolynomial.cc
#ifdef TRAJECTORY_TEST
#include "FourthDegreePolynomial.h"
#include "FifthDegreePolynomial.h"
#include <iostream>
int main() {
	double xInit = 0.0;
	double vInit = 0.0;
	double aInit = 0.0;
	double xFinal = 10.0;
	double vFinal = 10.0;
	double aFinal = 0.0;
	double duration = 5.0;
	artery::FourthDegreePolynomial fourthP(xInit, vInit, aInit, vFinal, aFinal, duration);
	artery::FifthDegreePolynomial fifthP(xInit, vInit, aInit, xFinal, vFinal, aFinal, duration);

	artery::Trajectory fourthT(fourthP, duration);
	artery::Trajectory fifthT(fifthP, duration);

	for (int i = 0; i < fourthT.getPoses().size(); i++) {
		std::cout << "pos: " << fourthT.getPoses().at(i) << " "
				  << "speed: " << fourthT.getSpeeds().at(i) << " "
				  << "accel: " << fourthT.getAccels().at(i) << " "
				  << "jerk: " << fourthT.getJerks().at(i) << std::endl;
	}

	for (int i = 0; i < fifthT.getPoses().size(); i++) {
		std::cout << "pos: " << fifthT.getPoses().at(i) << " "
				  << "speed: " << fifthT.getSpeeds().at(i) << " "
				  << "accel: " << fifthT.getAccels().at(i) << " "
				  << "jerk: " << fifthT.getJerks().at(i) << std::endl;
	}
}
#endif
