#include "FrenetPlanning.h"
#include "FrenetPath.h"
#include "Trajectory.h"
#include "QuinticPolynomial.h"
#include "QuarticPolynomial.h"
#include <vector>
#include <iostream>

namespace artery
{

FrenetPlanning::FrenetPlanning() {}

std::vector<FrenetPath> FrenetPlanning::generateFreePathCandidates(double mLonPos, double mLonSpeed, double mLonAccel,
    double mLatPos, double mLatSpeed, double mLatAccel,
    double mTargetLatMaxSpeed, std::vector<double> mTargetLatPoses, double mConvergenceTime) {
    
    std::vector<FrenetPath> pathCandidates;
    // 目標の横方向の位置を全探索
    for (double targetLatPos: mTargetLatPoses) {
        // 横方向の経路を５次元方程式から生成
        QuinticPolynomial lat_qp(mLatPos, mLatSpeed, mLatAccel, targetLatPos, 0.0, 0.0, mConvergenceTime);

        // 目標の縦方向の速度を、0から最大値まで全探索
        for (double targetLonSpeed = 0.0; targetLonSpeed <= mTargetLatMaxSpeed; targetLonSpeed += 0.1) {
            // 縦方向の経路を４次元方程式から生成
            QuarticPolynomial lon_qp(mLonPos, mLonSpeed, mLonAccel, targetLonSpeed, 0.0, mConvergenceTime);
            FrenetPath fp(lon_qp, lat_qp); 
            fp.calculateCost();
            pathCandidates.emplace_back(fp);
        }
    }

    return pathCandidates;
}

}

// ユニットテスト
// 実行方法
// mcsディレクトリ下で $g++ -DFRENETPLANNING_TEST FrenetPlanning.cc FrenetPath.cc Trajectory.cc QuinticPolynomial.cc QuarticPolynomial.cc
#ifdef FRENETPLANNING_TEST
int main() {
    artery::FrenetPlanning fp;
    double lon_pos = 0.0;
    double lon_speed = 10.0;
    double lon_accel = 0.0;
    double lat_pos = 0.0;
    double lat_speed = 0.0;
    double lat_accel = 0.0;
    double target_lon_max_speed = 10.0;
    std::vector<double> target_lat_poses = {-5.0, 0.0, 5.0};
    double convergence_time = 5.0;

    std::vector<artery::FrenetPath> freePathCandidates = fp.generateFreePathCandidates(
        lon_pos, lon_speed, lon_accel, 
        lat_pos, lat_speed, lat_accel, 
        target_lon_max_speed, target_lat_poses, convergence_time
    );


    for (auto path: freePathCandidates) {
        std::cout << "Lon speed: " << path.getLonTrajectory().getSpeeds().back() << 
            "m/s, Lat pos: " << path.getLatTrajectory().getPoses().back() << 
            "m, Cost: " << path.getCost() << std::endl;
    }
}
#endif