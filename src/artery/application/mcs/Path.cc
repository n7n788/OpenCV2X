#include "Trajectory.h"
#include "Path.h"
#include <vector>

namespace artery
{

void Path::calculateCost(double convergenceTime, double targetLonPos, double targetLonSpeed) {
    double lonJerkSquareSum = 0.0;
    double latJerkSquareSum = 0.0;
    double lonDiffPos = targetLonPos - mLonTrajectory.getPoses().back();
    double lonDiffSpeed = targetLonSpeed - mLonTrajectory.getSpeeds().back();
    // double latDiffPos = targetLatPos -  mLatTrajectory.getPoses().back();
    double lonCost;
    double latCost;

    for (int jerk: mLonTrajectory.getJerks()) {
        lonJerkSquareSum += jerk * jerk;
    }

    for (int jerk: mLatTrajectory.getJerks()) {
        latJerkSquareSum += jerk * jerk;
    }

    // 縦方向のコスト = ジャークの二乗和 + 目標スピードとの差分自乗
    lonCost = K_JERK * lonJerkSquareSum + 
              K_SPEED * lonDiffSpeed * lonDiffSpeed;
    // 横方向のコスト = ジャークの二乗和 + センターラインまでの距離の二乗
    latCost = K_JERK * latJerkSquareSum;
    mCost = K_LON * lonCost + K_LAT * latCost;
}
}

// ユニットテスト
// 実行方法
// mcsディレクトリ下で $g++ -DPATH_TEST Path.cc Trajectory.cc FifthDegreePolynomial.cc FourthDegreePolynomial.cc
#ifdef PATH_TEST
#include <iostream>
#include "FifthDegreePolynomial.h"
#include "FourthDegreePolynomial.h"
int main() {
    double targetLonSpeed = 10.0;
    double convergenceTime = 5.0;
    double duration = 5.0;
    artery::FourthDegreePolynomial lonP(0.0, 0.0, 0.0, targetLonSpeed, 0.0, convergenceTime);
    artery::FifthDegreePolynomial latP(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, convergenceTime);
    artery::Trajectory lonTrajectory(lonP, duration);
    artery::Trajectory latTrajectory(latP, duration);
    artery::Path path(lonTrajectory, latTrajectory);
    path.calculateCost(convergenceTime, 0.0, targetLonSpeed);
    std::cout << "Cost: " << path.getCost() << std::endl;
}
#endif
