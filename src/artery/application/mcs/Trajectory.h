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

	static constexpr double TIME_STEP = 0.1; // 軌跡のタイムステップ [s]
	static constexpr double TIME_LENGTH = 5.0; // 軌跡の時間長 [s]

	// getter
	const std::vector<double>& getPoses() const { return mPoses; };
	const std::vector<double>& getSpeeds() const { return mSpeeds; };
	const std::vector<double>& getAccels() const { return mAccels; };
	const std::vector<double>& getJerks() const { return mJerks; };
private:
	// TIME_STEP間隔で軌跡情報を生成
	std::vector<double> mPoses; // 位置配列 [m]
	std::vector<double> mSpeeds; // 速度配列 [m/s]
	std::vector<double> mAccels; // 加速度配列 [m/s^2]
	std::vector<double> mJerks; // ジャーク配列 [m/s^3]
};

}
#endif // ARTERY_TRAJECTORY_H
