#include "PathGenerator.h"
#include "Path.h"
#include "Trajectory.h"
#include "Polynomial.h"
#include "FourthDegreePolynomial.h"
#include "FifthDegreePolynomial.h"
#include <vector>
#include <iostream>

namespace artery
{

PathGenerator::PathGenerator() {}

std::vector<Path> PathGenerator::generateMaxSpeedPathCandidates(double lonPos, double lonSpeed, double lonAccel,
    double latPos, double latSpeed, double latAccel,
    double maxTargetLonSpeed, std::vector<double> targetLatPoses, double convergenceTime) {

    std::vector<Path> pathCandidates;
    // 横方向の目標位置を全探索
    for (double targetLatPos: targetLatPoses) {
        // 目標位置に達する横方向の経路を５次元方程式から生成
        Trajectory latTrajectory(FifthDegreePolynomial(latPos, latSpeed, latAccel, targetLatPos, 0.0, 0.0, convergenceTime), convergenceTime);

        // 縦方向の目標速度を、0から最大値まで全探索
        const double SPEED_STEP = 1.0;
        for (double targetLonSpeed = 0.0; targetLonSpeed <= maxTargetLonSpeed; targetLonSpeed += SPEED_STEP) {
            // 目標速度に達する縦方向の経路を４次元方程式から生成
            Trajectory lonTrajectory(FourthDegreePolynomial(lonPos, lonSpeed, lonAccel, targetLonSpeed, 0.0, convergenceTime), convergenceTime);
        
            Path fp(lonTrajectory, latTrajectory); 

            // 縦方向の目標速度と実際の終端速度の差分自乗をコストに加算
            fp.calculateCost(convergenceTime, lonTrajectory.getPoses().back(), maxTargetLonSpeed);
            pathCandidates.emplace_back(fp);
        }
    }

    return pathCandidates;
}

std::vector<Path> PathGenerator::generateMaxPosPathCandidates(double lonPos, double lonSpeed, double lonAccel,
    double latPos, double latSpeed, double latAccel,
    std::vector<double> targetLonPoses, std::vector<double> targetLonSpeeds, double maxTargetLonSpeed, 
    std::vector<double> targetLatPoses, double convergenceTime) {
    
    std::vector<Path> pathCandidates;
    std::size_t n = std::min({targetLonPoses.size(), targetLonSpeeds.size(), targetLatPoses.size()});
    
    // 各レーンの位置を全探索
    for (std::size_t i = 0; i < n; ++i) {
        double targetLonPos = targetLonPoses.at(i);
        double targetLonSpeed = targetLonSpeeds.at(i);
        double targetLatPos = targetLatPoses.at(i);

        // 横方向の経路を5次元方程式から生成
        Trajectory latTrajectory(FifthDegreePolynomial(latPos, latSpeed, latAccel, targetLatPos, 0.0, 0.0, convergenceTime), convergenceTime);

        // 縦方向の目標位置と速度に達する経路を5次元方程式から生成
        Trajectory lonTrajectory(FifthDegreePolynomial(lonPos, lonSpeed, lonAccel, targetLonPos, targetLonSpeed, 0.0, convergenceTime), convergenceTime);
    
        // 縦方向の目標位置と実際の終端位置の差分自乗をコストに加算
        Path fp(lonTrajectory, latTrajectory);
        fp.calculateCost(convergenceTime, targetLonPos, maxTargetLonSpeed);
        pathCandidates.emplace_back(fp);
    }

    return pathCandidates;
}

Path PathGenerator::generateSpeedPath(double lonPos, double lonSpeed, double lonAccel,
    double latPos, double latSpeed, double latAccel, double targetSpeed, double convergenceTime) {
    
    // 横方向の経路 (直進のみを) 5次元方程式から生成
    Trajectory latTrajectory(FifthDegreePolynomial(latPos, latSpeed, latAccel, latPos, 0.0, 0.0, convergenceTime), convergenceTime);
    // 縦方向の経路を4次元方程式から生成
    Trajectory lonTrajectory(FourthDegreePolynomial(lonPos, lonSpeed, lonAccel, targetSpeed, 0.0, convergenceTime), convergenceTime);
    
    Path fp(lonTrajectory, latTrajectory);
    
    return fp;
}
}

// ユニットテスト
// 実行方法
// mcsディレクトリ下で $g++ -DPATHGENERATOR_TEST PathGenerator.cc Path.cc Trajectory.cc FifthDegreePolynomial.cc FourthDegreePolynomial.cc
#ifdef PATHGENERATOR_TEST
void testGenerateMaxSpeedPathConditions() {
    artery::PathGenerator pathGenerator;
    double lonPos = 0.0;
    double lonSpeed = 10.0;
    double lonAccel = 0.0;
    double latPos = 0.0;
    double latSpeed = 0.0;
    double latAccel = 0.0;
    double targetLonSpeed = 16.0;
    std::vector<double> targetLatPoses = {5.0};
    double convergenceTime = 5.0;

    const std::vector<artery::Path> pathCandidates = pathGenerator.generateMaxSpeedPathCandidates(
        lonPos, lonSpeed, lonAccel, 
        latPos, latSpeed, latAccel, 
        targetLonSpeed, targetLatPoses, convergenceTime
    );

    std::cout << "max speed path candidates " << std::endl;
    for (auto path: pathCandidates) {
        std::cout << "End lon pos: " << path.getLonTrajectory().getPoses().back() << 
            "m, End lon speed: " << path.getLonTrajectory().getSpeeds().back() << 
            "m/s, End lat pos: " << path.getLatTrajectory().getPoses().back() <<
            "m, End lat speed: " << path.getLatTrajectory().getSpeeds().back() <<
            "m/s, Cost: " << path.getCost() << std::endl;
    }
}

void testGenerateMaxPosPathConditions() {
    artery::PathGenerator pg;
    double lonPos = 0.0;
    double lonSpeed = 5.0;
    double lonAccel = 0.0;
    double latPos = 0.0;
    double latSpeed = 0.0;
    double latAccel = 0.0;
    std::vector<double> maxTargetLonPoses = {20};
    std::vector<double> targetLonSpeeds = {5.0};
    double maxTargetLonSpeed = 5.0;
    std::vector<double> targetLatPoses = {5.0};
    double convergenceTime = 5.0;

    const std::vector<artery::Path> followingPathCandidates = pg.generateMaxPosPathCandidates(
        lonPos, lonSpeed, lonAccel, 
        latPos, latSpeed, latAccel, 
        maxTargetLonPoses, targetLonSpeeds, maxTargetLonSpeed, 
        targetLatPoses, convergenceTime
    );

    std::cout << "max pos path candidates " << std::endl;
    for (auto path: followingPathCandidates) {
        std::cout << "End lon pos: " << path.getLonTrajectory().getPoses().back() << 
            "m, End lon speed: " << path.getLonTrajectory().getSpeeds().back() << 
            "m/s, End lat pos: " << path.getLatTrajectory().getPoses().back() <<
            "m, End lat speed: " << path.getLatTrajectory().getSpeeds().back() <<
            "m/s, Cost: " << path.getCost() << std::endl;
    }
}

int main() {
    testGenerateMaxSpeedPathConditions();
    testGenerateMaxPosPathConditions();
}
#endif

