#ifndef ARTERY_PATH_H
#define ARTERY_PATH_H

#include "Trajectory.h" 
#include <vector>

namespace artery
{

class Path
{
public:
	/**
	 * @param lonTrajectory [Trajectory] 縦方向の経路
	 * @param latTrajectory [Trajectory] 横方向の経路
	*/
	Path(const Trajectory& lonTrajectory, const Trajectory& latTrajectory) : 
		mLonTrajectory(lonTrajectory),
		mLatTrajectory(latTrajectory),
		mCost(0.0) {}
	

	Path() : mCost(-1.0) {}

	static constexpr double K_JERK = 0.1; // ジャークの重み
	static constexpr double K_SPEED = 5.0; // 速度の重み
	static constexpr double K_LON = 1.0; // 縦方向の重み
	static constexpr double K_LAT = 1.0; // 横方向の重み

	// getter 
	const artery::Trajectory& getLonTrajectory() const { return mLonTrajectory; }
	const artery::Trajectory& getLatTrajectory() const { return mLatTrajectory; }
	double getCost() const { return mCost; }

	// setter
	void setLonTrajectory(const artery::Trajectory& lonTrajectory) { mLonTrajectory = lonTrajectory; }
	void setLatTrajectory(const artery::Trajectory& latTrajectory) { mLatTrajectory = latTrajectory; }

	// 経路のコストを計算
	// @param targetLonPos [double] 目標の縦方向の位置 m
	// @param targetLonSpeed [double] 目標の縦方向の速度 m/s
	void calculateCost(double targetLonPos, double targetLonSpeed);

private:
	Trajectory mLonTrajectory; // 縦方向のパス
	Trajectory mLatTrajectory; // 横方向のパス
	double mCost{}; // 経路のコスト
};
}
#endif /* ARTERY_PATH_H */
