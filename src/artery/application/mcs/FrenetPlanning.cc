#include "FrenetPlanning.h"
#include "FrenetPath.h"
#include "Trajectory.h"
#include "Polynomial.h"
#include "FourthDegreePolynomial.h"
#include "FifthDegreePolynomial.h"
#include <vector>
#include <iostream>

namespace artery
{

FrenetPlanning::FrenetPlanning() {}

std::vector<FrenetPath> FrenetPlanning::generateMaxSpeedPathCandidates(double latPos, double latSpeed, double latAccel,
    double lonPos, double lonSpeed, double lonAccel,
    double maxTargetLatSpeed, std::vector<double> targetLonPoses, double convergenceTime) {

    std::vector<FrenetPath> pathCandidates;
    // 横方向の目標位置を全探索
    for (double targetLonPos: targetLonPoses) {
        // 目標位置に達する横方向の経路を５次元方程式から生成
        Trajectory lonTrajectory(FifthDegreePolynomial(lonPos, lonSpeed, lonAccel, targetLonPos, 0.0, 0.0, convergenceTime), convergenceTime);

        // 縦方向の目標速度を、0から最大値まで全探索
        const double SPEED_STEP = 1.0;
        for (double targetLatSpeed = 0.0; targetLatSpeed <= maxTargetLatSpeed; targetLatSpeed += SPEED_STEP) {
            // 目標速度に達する縦方向の経路を４次元方程式から生成
            Trajectory latTrajectory(FourthDegreePolynomial(latPos, latSpeed, latAccel, targetLatSpeed, 0.0, convergenceTime), convergenceTime);
            FrenetPath fp(latTrajectory, lonTrajectory); 

            // 縦方向の目標速度と実際の終端速度の差分自乗をコストに加算
            fp.calculateCost(convergenceTime, latTrajectory.getPoses().back(), maxTargetLatSpeed);
            pathCandidates.emplace_back(fp);
        }
    }

    return pathCandidates;
}

std::vector<FrenetPath> FrenetPlanning::generateMaxPosPathCandidates(double latPos, double latSpeed, double latAccel,
    double lonPos, double lonSpeed, double lonAccel,
    std::vector<double> targetLatPoses, std::vector<double> targetLatSpeeds, double maxTargetLatSpeed, 
    std::vector<double> targetLonPoses, double convergenceTime) {
    
    std::vector<FrenetPath> pathCandidates;
    std::size_t n = std::min({targetLatPoses.size(), targetLatSpeeds.size(), targetLonPoses.size()});
    
    // 横方向の位置を全探索
    for (std::size_t i = 0; i < n; ++i) {
        double targetLatPos = targetLatPoses.at(i);
        double targetLatSpeed = targetLatSpeeds.at(i);
        double targetLonPos = targetLonPoses.at(i);

        // 横方向の経路を5次元方程式から生成
        Trajectory lonTrajectory(FifthDegreePolynomial(lonPos, lonSpeed, lonAccel, targetLonPos, 0.0, 0.0, convergenceTime), convergenceTime);

        // 縦方向の目標位置と速度に達する経路を5次元方程式から生成
        Trajectory latTrajectory(FifthDegreePolynomial(latPos, latSpeed, latAccel, targetLatPos, targetLatSpeed, 0.0, convergenceTime), convergenceTime);
    
        // 縦方向の目標位置と実際の終端位置の差分自乗をコストに加算
        FrenetPath fp(latTrajectory, lonTrajectory);
        fp.calculateCost(convergenceTime, targetLatPos, maxTargetLatSpeed);
        pathCandidates.emplace_back(fp);
    }

    return pathCandidates;
}

FrenetPath FrenetPlanning::generateSpeedPath(double latPos, double latSpeed, double latAccel,
    double lonPos, double lonSpeed, double lonAccel, double targetSpeed, double convergenceTime) {
    
    // 横方向の経路 (直進のみを) 5次元方程式から生成
    Trajectory lonTrajectory(FifthDegreePolynomial(lonPos, lonSpeed, lonAccel, lonPos, 0.0, 0.0, convergenceTime), convergenceTime);
    // 縦方向の経路を4次元方程式から生成
    Trajectory latTrajectory(FourthDegreePolynomial(latPos, latSpeed, latAccel, targetSpeed, 0.0, convergenceTime), convergenceTime);
    
    FrenetPath fp(latTrajectory, lonTrajectory);
    
    return fp;
}
}

// ユニットテスト
// 実行方法
// mcsディレクトリ下で $g++ -DFRENETPLANNING_TEST FrenetPlanning.cc FrenetPath.cc Trajectory.cc QuinticPolynomial.cc QuarticPolynomial.cc
#ifdef FRENETPLANNING_TEST

void testGenerateMaxSpeedPathConditions() {
    artery::FrenetPlanning fp;
    double latPos = 0.0;
    double latSpeed = 10.0;
    double latAccel = 0.0;
    double lonPos = 0.0;
    double lonSpeed = 0.0;
    double lonAccel = 0.0;
    double targetLatSpeed = 16.0;
    std::vector<double> targetLonPoses = {5.0};
    double convergenceTime = 5.0;

    const std::vector<artery::FrenetPath> freePathCandidates = fp.generateMaxSpeedPathCandidates(
        latPos, latSpeed, latAccel, 
        lonPos, lonSpeed, lonAccel, 
        targetLatSpeed, targetLonPoses, convergenceTime
    );

    std::cout << "max speed path candidates " << std::endl;
    for (auto path: freePathCandidates) {
        std::cout << "End lat pos: " << path.getLatTrajectory().getPoses().back() << 
            "m, End lat speed: " << path.getLatTrajectory().getSpeeds().back() << 
            "m/s, End lon pos: " << path.getLonTrajectory().getPoses().back() <<
            "m, End lon speed: " << path.getLatTrajectory().getSpeeds().back() <<
            "m/s, Cost: " << path.getCost() << std::endl;
    }
}

void testGenerateMaxPosPathConditions() {
    artery::FrenetPlanning fp;
    double latPos = 0.0;
    double latSpeed = 5.0;
    double latAccel = 0.0;
    double lonPos = 0.0;
    double lonSpeed = 0.0;
    double lonAccel = 0.0;
    std::vector<double> maxTargetLatPoses = {20};
    std::vector<double> targetLatSpeeds = {5.0};
    double maxTargetLatSpeed = 5.0;
    std::vector<double> targetLonPoses = {5.0};
    double convergenceTime = 5.0;

    const std::vector<artery::FrenetPath> followingPathCandidates = fp.generateMaxPosPathCandidates(
        latPos, latSpeed, latAccel, 
        lonPos, lonSpeed, lonAccel, 
        maxTargetLatPoses, targetLatSpeeds, maxTargetLatSpeed, 
        targetLonPoses, convergenceTime
    );

    std::cout << "max pos path candidates " << std::endl;
    for (auto path: followingPathCandidates) {
        std::cout << "End lat pos: " << path.getLatTrajectory().getPoses().back() << 
            "m, End lat speed: " << path.getLatTrajectory().getSpeeds().back() << 
            "m/s, End lon pos: " << path.getLonTrajectory().getPoses().back() <<
            "m, End lon speed: " << path.getLonTrajectory().getSpeeds().back() <<
            "m/s, Cost: " << path.getCost() << std::endl;
    }
}

int main() {
    testGenerateMaxSpeedPathConditions();
    testGenerateMaxPosPathConditions();
}
#endif
