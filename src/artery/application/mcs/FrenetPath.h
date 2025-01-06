#ifndef ARTERY_FRENETPATH_H
#define ARTERY_FRENETPATH_H

#include "Trajectory.h" 
#include <vector>
namespace artery
{

class FrenetPath
{
public:
	// デフォルトコンストラクタ
	FrenetPath(const QuarticPolynomial& lonPolynomial, const QuinticPolynomial& latPolynomial) {
		mLonTrajectory = Trajectory(lonPolynomial);
		mLatTrajectory = Trajectory(latPolynomial);
	}

	static constexpr double K_JERK = 0.1; // ジャークの重み
	static constexpr double K_TIME = 1.0; // 時間の重み
	static constexpr double K_DISTANCE = 3.0; // 距離の重み
	static constexpr double K_SPEED = 3.0; // 速度の重み
	static constexpr double K_LAT = 1.0; // 横方向の重み
	static constexpr double K_LON = 1.0; // 縦方向の重み

	// getter 
	const artery::Trajectory& getLonTrajectory() { return mLonTrajectory; }
	const artery::Trajectory& getLatTrajectory() { return mLatTrajectory; }
	double getCost() { return mCost; }

	// setter
	void setLonTrajectory(const artery::Trajectory& lonTrajectory) { mLonTrajectory = lonTrajectory; }
	void setLatTrajectory(const artery::Trajectory& latTrajectory) { mLatTrajectory = latTrajectory; }

	// 経路のコストを計算
	void calculateCost();

private:
	Trajectory mLonTrajectory; // 縦方向のパス
	Trajectory mLatTrajectory; // 横方向のパス
	double mCost; // 経路のコスト
};
}
#endif /* ARTERY_FRENETPATH_H */
