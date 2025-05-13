
#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"
#include "Trajectory.h"
#include "FrenetPath.h"
#include <vector>
#include <iostream>

namespace artery
{

void FrenetPath::calculateCost(double convergenceTime, double targetLatPos, double targetLatSpeed) {
    double latJerkSquareSum = 0.0;
    double lonJerkSquareSum = 0.0;
    double latDiffPos = targetLatPos - mLatTrajectory.getPoses().back();
    double latDiffSpeed = targetLatSpeed - mLatTrajectory.getSpeeds().back();
    // double lonDiffPos = targetLonPos -  mLonTrajectory.getPoses().back();
    double latCost;
    double lonCost;

    for (int jerk: mLatTrajectory.getJerks()) {
        latJerkSquareSum += jerk * jerk;
    }

    for (int jerk: mLonTrajectory.getJerks()) {
        lonJerkSquareSum += jerk * jerk;
    }

    // 縦方向のコスト = ジャークの二乗和 + 収束時間 + 目標位置との差分自乗 + 目標スピードとの差分自乗
    latCost = K_JERK * latJerkSquareSum + 
              K_SPEED * latDiffSpeed * latDiffSpeed;
            //   K_TIME * (5.0 - convergenceTime) + 
            //   K_DISTANCE * latDiffPos * latDiffPos +
    // 横方向のコスト = ジャークの二乗和 + 収束時間 + センターラインまでの距離の二乗
    lonCost = K_JERK * lonJerkSquareSum;
            //   K_TIME * convergenceTime;
            //   K_DISTANCE * mLonTrajectory.getPoses().back() * mLonTrajectory.getPoses().back(); 
    mCost = K_LAT * latCost + K_LON * lonCost;
}
}

// ユニットテスト
// 実行方法
// mcsディレクトリ下で $g++ -DFRENETPATH_TEST FrenetPath.cc Trajectory.cc QuinticPolynomial.cc QuarticPolynomial.cc
#ifdef FRENETPATH_TEST
int main() {
    double targetLatSpeed = 5.0;
    double convergenceTime = 5.0;
    artery::QuarticPolynomial latQp(0.0, 0.0, 0.0, targetLatSpeed, 0.0, convergenceTime);
    artery::QuinticPolynomial lonQp(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, convergenceTime);
    artery::Trajectory latTrajectory(latQp);
    artery::Trajectory lonTrajectory(lonQp);
    artery::FrenetPath fp(latTrajectory, lonTrajectory);
    fp.calculateCost(targetLatSpeed, convergenceTime);
    std::cout << "Cost: " << fp.getCost() << std::endl;
}
#endif
