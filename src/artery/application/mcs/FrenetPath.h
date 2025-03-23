#ifndef ARTERY_FRENETPATH_H
#define ARTERY_FRENETPATH_H

#include "Trajectory.h" 
#include <vector>
namespace artery
{

class FrenetPath
{
public:
	// @param latTrajectory [Trajectory] 縦方向の経路
	// @param lonTrajectory [Trajectory] 横方向の経路
	FrenetPath(const Trajectory& latTrajectory, const Trajectory& lonTrajectory) {
		mLatTrajectory = latTrajectory;
		mLonTrajectory = lonTrajectory;
	}

	FrenetPath() {}

	static constexpr double K_JERK = 0.1; // ジャークの重み
	static constexpr double K_TIME = 1.0; // 時間の重み
	static constexpr double K_DISTANCE = 3.0; // 距離の重み
	static constexpr double K_SPEED = 3.0; // 速度の重み
	static constexpr double K_LAT = 1.0; // 横方向の重み
	static constexpr double K_LON = 1.0; // 縦方向の重み

	// getter 
	const artery::Trajectory& getLatTrajectory() const { return mLatTrajectory; }
	const artery::Trajectory& getLonTrajectory() const { return mLonTrajectory; }
	double getCost() const { return mCost; }

	// setter
	void setLatTrajectory(const artery::Trajectory& latTrajectory) { mLatTrajectory = latTrajectory; }
	void setLonTrajectory(const artery::Trajectory& lonTrajectory) { mLonTrajectory = lonTrajectory; }

	// 経路のコストを計算
	// @param convergenceTime [double] 収束時間 s
	// @param targetLatPos [double] 目標の縦方向の位置 m
	// @param targetLatSpeed [double] 目標の縦方向の速度 m/s
	void calculateCost(double convergenceTime, double targetLatPos, double targetLatSpeed);

private:
	Trajectory mLatTrajectory; // 縦方向のパス
	Trajectory mLonTrajectory; // 横方向のパス
	double mCost; // 経路のコスト
};
}
#endif /* ARTERY_FRENETPATH_H */
