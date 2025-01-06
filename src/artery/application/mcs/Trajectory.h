#ifndef ARTERY_TRAJECTORY_H
#define ARTERY_TRAJECTORY_H


#include "QuinticPolynomial.h"
#include "QuarticPolynomial.h"
#include <vector>

namespace artery
{

// 1次元の軌跡情報を格納するクラス
class Trajectory
{
public:

	// デフォルトコンストラクタ
	Trajectory() {}

	// コンストラクタ
	// 5次元方程式から軌跡情報を生成
	// @param qp [QuinticPolynomial] 5次元方程式
	Trajectory(const QuinticPolynomial& qp);

	// 4次元方程式から軌跡情報を生成
	// @param qp [QuarticPolynomial] 4次元方程式
	Trajectory(const QuarticPolynomial& qp);

	static constexpr double TIME_STEP = 0.1; // 軌跡のタイムステップ
	static constexpr double TIME_LENGTH = 5.0; // 軌跡の時間長

	// getter
	const std::vector<double>& getPoses() { return mPoses; };
	const std::vector<double>& getSpeeds() { return mSpeeds; };
	const std::vector<double>& getAccels() { return mAccels; };
	const std::vector<double>& getJerks() { return mJerks; };
	const double getConvergenceTime() { return mConvergenceTime; };
	const double getTargetSpeed() { return mTargetSpeed; };
private:
  
	std::vector<double> mPoses; // 位置配列
	std::vector<double> mSpeeds; // 速度配列
	std::vector<double> mAccels; // 加速度配列
	std::vector<double> mJerks; // ジャーク配列
	double mConvergenceTime; // 収束時間
	double mTargetSpeed; // 目標速度
};

}
#endif // ARTERY_TRAJECTORY_H
