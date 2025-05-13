#ifndef ARTERY_TRAJECTORY_H
#define ARTERY_TRAJECTORY_H

#include "Polynomial.h"
#include <vector>

namespace artery
{

/**
 * @brief 軌跡
 * 
 * n次多項式を受け取り、各タイムステップにおける位置、速度、加速度、ジャークを計算するクラス
*/
class Trajectory
{
public:

	Trajectory() {}

    /**
	 *  @param p [Polynomial] n次元方程式
	 */
	Trajectory(const Polynomial& p, double duration);

	static constexpr double TIME_STEP = 0.1; // 軌跡のタイムステップ [s]

	// getter
	const std::vector<double>& getPoses() const { return mPoses; };
	const std::vector<double>& getSpeeds() const { return mSpeeds; };
	const std::vector<double>& getAccels() const { return mAccels; };
	const std::vector<double>& getJerks() const { return mJerks; };
private:
	// TIME_STEP間隔で軌跡情報を生成
	double mDuration; // 軌跡の長さ [s]
	std::vector<double> mPoses; // 位置配列 [m]
	std::vector<double> mSpeeds; // 速度配列 [m/s]
	std::vector<double> mAccels; // 加速度配列 [m/s^2]
	std::vector<double> mJerks; // ジャーク配列 [m/s^3]
};

}
#endif // ARTERY_TRAJECTORY_H
