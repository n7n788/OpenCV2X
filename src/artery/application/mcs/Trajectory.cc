#include "Trajectory.h"
#include "Polynomial.h"
#include <vector>
#include <cmath>
#include <cassert>

namespace artery
{

Trajectory::Trajectory(const artery::Polynomial& p, 
					   double initTime,
					   double duration) 
	: mInitTime(initTime),
	  mDuration(duration)
{
	assert(duration > 0.0 && initTime >= 0.0);

	// emplace_backでメモリの再確保を防ぐため、あらかじめreserveしておく
	std::size_t nPts = static_cast<std::size_t>(std::ceil(duration / TIME_STEP));
	mPoses.reserve(nPts);
	mSpeeds.reserve(nPts);
	mAccels.reserve(nPts);
	mJerks.reserve(nPts);

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
	double initTime = 5.0;
	double duration = 5.0;
	artery::FourthDegreePolynomial fourthP(xInit, vInit, aInit, vFinal, aFinal, duration);
	artery::FifthDegreePolynomial fifthP(xInit, vInit, aInit, xFinal, vFinal, aFinal, duration);

	artery::Trajectory fourthT(fourthP, initTime, duration);
	artery::Trajectory fifthT(fifthP, initTime, duration);

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
