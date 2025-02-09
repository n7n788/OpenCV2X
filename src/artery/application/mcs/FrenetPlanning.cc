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

std::vector<FrenetPath> FrenetPlanning::generateFreePathCandidates(double latPos, double latSpeed, double latAccel,
    double lonPos, double lonSpeed, double lonAccel,
    double maxTargetLatSpeed, std::vector<double> targetLonPoses, double convergenceTime) {
    
    std::vector<FrenetPath> pathCandidates;
    // 目標の横方向の位置を全探索
    for (double targetLonPos: targetLonPoses) {
        // 横方向の経路を５次元方程式から生成
        Trajectory lonTrajectory(QuinticPolynomial(lonPos, lonSpeed, lonAccel, targetLonPos, 0.0, 0.0, convergenceTime));

        // 目標の縦方向の速度を、0から最大値まで全探索
        const double SPEED_STEP = 0.1;
        for (double targetLatSpeed = 0.0; targetLatSpeed <= maxTargetLatSpeed; targetLatSpeed += SPEED_STEP) {
            // 縦方向の経路を４次元方程式から生成
            Trajectory latTrajectory(QuarticPolynomial(latPos, latSpeed, latAccel, targetLatSpeed, 0.0, convergenceTime));
            FrenetPath fp(latTrajectory, lonTrajectory); 

            // 縦方向の目標速度と実際の終端速度の差分自乗をコストに加算
            fp.calculateCost(convergenceTime, latTrajectory.getPoses().back(), maxTargetLatSpeed);
            pathCandidates.emplace_back(fp);
        }
    }

    return pathCandidates;
}

std::vector<FrenetPath> FrenetPlanning::generateFollowingPathCandidates(double latPos, double latSpeed, double latAccel,
    double lonPos, double lonSpeed, double lonAccel,
    double maxTargetLatPos, double targetLatSpeed, double convergenceTime) {
    
    std::vector<FrenetPath> pathCandidates;
    
    // 横方向の位置を現在の車線に固定して経路を生成
    Trajectory lonTrajectory(QuinticPolynomial(lonPos, lonSpeed, lonAccel, 0.0, 0.0, 0.0, convergenceTime));

    // 目標の縦方向の速度を固定し、目標位置を最大位置からデクリメントして探索
    const double POS_STEP = 10; // 10mずつ探索
    for (double targetLatPos = maxTargetLatPos; targetLatPos >= latPos; targetLatPos -= POS_STEP) {
        // 縦方向の経路を４次元方程式から生成
        Trajectory latTrajectory(QuinticPolynomial(latPos, latSpeed, latAccel, targetLatPos, targetLatSpeed, 0.0, convergenceTime));
        FrenetPath fp(latTrajectory, lonTrajectory);

        // 縦方向の目標位置と実際の終端位置の差分自乗をコストに加算
        fp.calculateCost(convergenceTime, targetLatPos, targetLatSpeed);
        pathCandidates.emplace_back(fp);
    }

    return pathCandidates;
}
}

artery::FrenetPath artery::FrenetPlanning::selectMinCostPath(const std::vector<FrenetPath>& pathCandidates) {
    double minCost = std::numeric_limits<double>::max();

    FrenetPath minCostPath;
    for (const FrenetPath& path: pathCandidates) {
        if (path.getCost() < minCost) {
            minCost = path.getCost();
            minCostPath = path;
        }
    }
    return minCostPath;
}

// ユニットテスト
// 実行方法
// mcsディレクトリ下で $g++ -DFRENETPLANNING_TEST FrenetPlanning.cc FrenetPath.cc Trajectory.cc QuinticPolynomial.cc QuarticPolynomial.cc
#ifdef FRENETPLANNING_TEST

void testFreePathConditions() {
    artery::FrenetPlanning fp;
    double latPos = 0.0;
    double latSpeed = 5.0;
    double latAccel = 0.0;
    double lonPos = 0.0;
    double lonSpeed = 0.0;
    double lonAccel = 0.0;
    double targetLatSpeed = 5.0;
    std::vector<double> targetLonPoses = {5.0};
    double convergenceTime = 5.0;

    const std::vector<artery::FrenetPath> freePathCandidates = fp.generateFreePathCandidates(
        latPos, latSpeed, latAccel, 
        lonPos, lonSpeed, lonAccel, 
        targetLatSpeed, targetLonPoses, convergenceTime
    );

    std::cout << "Free path candidates" << std::endl;
    for (auto path: freePathCandidates) {
        std::cout << "End lat pos: " << path.getLatTrajectory().getPoses().back() << 
            "m/s, End lon pos: " << path.getLonTrajectory().getPoses().back() <<
            "m, End lon speed: " << path.getLonTrajectory().getSpeeds().back() <<
            "m/s, Cost: " << path.getCost() << std::endl;
    }

    artery::FrenetPath minCostPath = fp.selectMinCostPath(freePathCandidates);
    std::cout << "Min cost path" << std::endl;
    std::cout << "End lat pos: " << minCostPath.getLatTrajectory().getPoses().back() << 
        "m/s, End lon pos: " << minCostPath.getLonTrajectory().getPoses().back() <<
        "m, End lon speed: " << minCostPath.getLonTrajectory().getSpeeds().back() <<
        "m/s, Cost: " << minCostPath.getCost() << std::endl;
}

void testFollowingPathConditions() {
    artery::FrenetPlanning fp;
    double latPos = 0.0;
    double latSpeed = 5.0;
    double latAccel = 0.0;
    double lonPos = 0.0;
    double lonSpeed = 0.0;
    double lonAccel = 0.0;
    double maxTargetLatPos = 20;
    double targetLatSpeed = 5.0;
    double convergenceTime = 5.0;

    const std::vector<artery::FrenetPath> followingPathCandidates = fp.generateFollowingPathCandidates(
        latPos, latSpeed, latAccel, 
        lonPos, lonSpeed, lonAccel, 
        maxTargetLatPos, targetLatSpeed, convergenceTime
    );

    std::cout << "Following path candidates" << std::endl;
    for (auto path: followingPathCandidates) {
        std::cout << "End lat pos: " << path.getLatTrajectory().getPoses().back() << 
            "m, End lat speed: " << path.getLatTrajectory().getSpeeds().back() << 
            "m/s, End lon pos: " << path.getLonTrajectory().getPoses().back() <<
            "m, End lon speed: " << path.getLonTrajectory().getSpeeds().back() <<
            "m/s, Cost: " << path.getCost() << std::endl;
    }
    
    artery::FrenetPath minCostPath = fp.selectMinCostPath(followingPathCandidates);
    std::cout << "Min cost path" << std::endl;
    std::cout << "End lat pos: " << minCostPath.getLatTrajectory().getPoses().back() << 
        "m/s, End lon pos: " << minCostPath.getLonTrajectory().getPoses().back() <<
        "m, End lon speed: " << minCostPath.getLonTrajectory().getSpeeds().back() <<
        "m/s, Cost: " << minCostPath.getCost() << std::endl;
}

int main() {
    testFreePathConditions();
    testFollowingPathConditions();
}
#endif
