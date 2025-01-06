
#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"
#include "Trajectory.h"
#include "FrenetPath.h"
#include <vector>
#include <iostream>

namespace artery
{

void FrenetPath::calculateCost() {
    double latJerkSquareSum = 0.0;
    double lonJerkSquareSum = 0.0;
    double latDiffSpeed = mLatTrajectory.getTargetSpeed() - mLatTrajectory.getSpeeds().back();
    double latCost;
    double lonCost;

    for (int jerk: mLatTrajectory.getJerks()) {
        latJerkSquareSum += jerk * jerk;
    }

    for (int jerk: mLonTrajectory.getJerks()) {
        lonJerkSquareSum += jerk * jerk;
    }

    // 縦方向のコスト = ジャークの二乗和 + 収束時間 + 目標スピードとの差分自乗
    latCost = K_JERK * latJerkSquareSum + 
              K_TIME * mLatTrajectory.getConvergenceTime() + 
              K_SPEED * latDiffSpeed * latDiffSpeed;
    // 横方向のコスト = ジャークの二乗和 + 収束時間 + センターラインまでの距離の二乗
    lonCost = K_JERK * lonJerkSquareSum + 
              K_TIME * mLonTrajectory.getConvergenceTime() + 
              K_DISTANCE * mLonTrajectory.getPoses().back() * mLonTrajectory.getPoses().back(); 
    mCost = K_LAT * latCost + K_LON * lonCost;
}
}

// ユニットテスト
// 実行方法
// mcsディレクトリ下で $g++ -DFRENETPATH_TEST FrenetPath.cc Trajectory.cc QuinticPolynomial.cc QuarticPolynomial.cc
#ifdef FRENETPATH_TEST
int main() {
    artery::QuinticPolynomial qp1(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 5.0);
    artery::QuarticPolynomial qp2(0.0, 0.0, 0.0, 2.0, 0.0, 5.0);
    artery::FrenetPath fp(qp2, qp1);
    fp.calculateCost();
    std::cout << "Cost: " << fp.getCost() << std::endl;
}
#endif
