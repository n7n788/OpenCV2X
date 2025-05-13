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

    /**
	 *  @param p [Polynomial] n次元方程式
	 *  @param initTime [double] 軌跡の初期時刻 [s]
	 *  @param duration [double] 軌跡の時間幅 [s]
	 */
	Trajectory(const Polynomial&, 
			   double initTime, 
			   double duration);

	static constexpr double TIME_STEP = 0.1; // 軌跡のタイムステップ [s]

	// getter
	const std::vector<double>& getPoses() const noexcept { return mPoses; }
	const std::vector<double>& getSpeeds() const noexcept { return mSpeeds; }
	const std::vector<double>& getAccels() const noexcept { return mAccels; }
	const std::vector<double>& getJerks() const noexcept { return mJerks; }

private:
	double mInitTime{}; // 軌跡の初期時刻 [s]
	double mDuration{}; // 軌跡の時間幅 [s]
	// TIME_STEP間隔で軌跡情報を生成
	std::vector<double> mPoses{}; // 位置配列 [m]
	std::vector<double> mSpeeds{}; // 速度配列 [m/s]
	std::vector<double> mAccels{}; // 加速度配列 [m/s^2]
	std::vector<double> mJerks{}; // ジャーク配列 [m/s^3]
};

}
#endif // ARTERY_TRAJECTORY_H
