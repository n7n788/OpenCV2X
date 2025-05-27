#ifndef ARTERY_PATHGENERATOR_H
#define ARTERY_PATHGENERATOR_H

#include "Path.h"
#include <vector>

namespace artery
{

class PathGenerator
{
public:
    PathGenerator();

    /**
        * 自由走行の経路を生成 
        *   縦方向の最高速度を指定し、各レーンにおける経路の候補を生成 
        * @param lonPos [double]   現在の縦方向の位置 [m] 
        * @param lonSpeed [double] 現在の縦方向の速度 [m/s]
        * @param lonAccel [double] 現在の縦方向の加速度 [m/s^2] 
        * @param latPos [double]   現在の横方向の位置 [m] 
        * @param latSpeed [double] 現在の横方向の速度 [m/s] 
        * @param latAccel [double] 現在の横方向の加速度 [m/s^2] 
        * @param maxTargetLonSpeed [double] 目標の縦方向の速度の最大値 [m/s]
        * @param targetLatPoses [vector<double>]  目標の横方向の位置 [m] 
        * @param convergenceTime [double] 収束時間 [s] 
    */
    std::vector<Path> generateMaxSpeedPathCandidates(double lonPos, double lonSpeed, double lonAccel, 
        double latPos, double latSpeed, double latAccel, 
        double maxTargetLonSpeed, std::vector<double> targetLatPoses, double convergenceTime);
    
    /**
        * 前方車両に追従走行する経路を生成  
        *   各レーンで縦方向の位置と速度を指定 
        * @param lonPos [double]   現在の縦方向の位置 [m] 
        * @param lonSpeed [double] 現在の縦方向の速度 [m/s]
        * @param lonAccel [double] 現在の縦方向の加速度 [m/s^2] 
        * @param latPos [double]   現在の横方向の位置 [m] 
        * @param latSpeed [double] 現在の横方向の速度 [m/s] 
        * @param latAccel [double] 現在の横方向の加速度 [m/s^2] 
        * @param targetLonPoses [vector<double>] 目標の縦方向の位置 [m]
        * @param targetLonSpeeds [vector<double>]   目標の縦方向の速度 [m/s]
        * @param maxTargetLonSpeed [double] 目標の縦方向の速度の最大値 [m/s] コスト計算で使用
        * @param targetLatPoses [vector<double>]  目標の横方向の位置 [m] 
        * @param convergenceTime [double] 収束時間 [s] 
    */
    std::vector<Path> generateMaxPosPathCandidates(double lonPos, double lonSpeed, double lonAccel, 
        double latPos, double latSpeed, double latAccel, 
        std::vector<double> targetLonPoses, std::vector<double> targetLonSpeeds, double maxTargetLonSpeed,
        std::vector<double> targetLatPoses, double convergenceTime);

    /**
        * ある速度に到達する直進経路を生成
        * @param lonPos [double]   現在の縦方向の位置 [m]
        * @param lonSpeed [double] 現在の縦方向の速度 [m/s]
        * @param lonAccel [double] 現在の縦方向の加速度 [m/s^2] 
        * @param latPos [double]   現在の横方向の位置 [m] 
        * @param latSpeed [double] 現在の横方向の速度 [m/s] 
        * @param latAccel [double] 現在の横方向の加速度 [m/s^2] 
        * @param targetSpeed [double] 目標の速度 [m/s]
        * @param convergenceTime [double] 収束時間 [s] 
    */
    Path generateSpeedPath(double lonPos, double lonSpeed, double lonAccel, 
        double latPos, double latSpeed, double latAccel, double targetSpeed, double convergenceTime);

};
}
#endif /* ARTERY_PATHGENERATOR_H */
