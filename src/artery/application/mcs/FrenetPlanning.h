#ifndef ARTERY_FRENETPLANNING_H
#define ARTERY_FRENETPLANNING_H

#include "FrenetPath.h"
#include <vector>

namespace artery
{

class FrenetPlanning
{
public:
    FrenetPlanning();

    /**
        * 縦方向の最高速度を指定し、各レーンにおける経路の候補を生成 
        *  目標位置は自由
        * @param latPos [double]   現在の縦方向の位置 [m] 
        * @param latSpeed [double] 現在の縦方向の速度 [m/s]
        * @param latAccel double] 現在の縦方向の加速度 [m/s^2] 
        * @param lonPos [double]   現在の横方向の位置 [m] 
        * @param lonSpeed [double] 現在の横方向の速度 [m/s] 
        * @param lonAccel [double] 現在の横方向の加速度 [m/s^2] 
        * @param maxTargetLatSpeed [double] 目標の縦方向の速度の最大値 [m/s]
        * @param targetLonPoses [vector<double>]  目標の横方向の位置 [m] 
        * @param convergenceTime [double] 収束時間 [s] 
    */
    std::vector<FrenetPath> generateMaxSpeedPathCandidates(double latPos, double latSpeed, double latAccel, 
        double lonPos, double lonSpeed, double lonAccel, 
        double maxTargetLatSpeed, std::vector<double> targetLonPoses, double convergenceTime);
    
    /**
        * 各レーンで縦方向の位置と速度を指定し、経路の候補を生成 
        * @param latPos [double]   現在の縦方向の位置 [m] 
        * @param latSpeed [double] 現在の縦方向の速度 [m/s]
        * @param latAccel double] 現在の縦方向の加速度 [m/s^2] 
        * @param lonPos [double]   現在の横方向の位置 [m] 
        * @param lonSpeed [double] 現在の横方向の速度 [m/s] 
        * @param lonAccel [double] 現在の横方向の加速度 [m/s^2] 
        * @param targetLatPoses [vector<double>] 目標の縦方向の位置 [m]
        * @param targetLatSpeeds [vector<double>]   目標の縦方向の速度 [m/s]
        * @param maxTargetLatSpeed [double] 目標の縦方向の速度の最大値 [m/s] コスト計算で使用
        * @param targetLonPoses [vector<double>]  目標の横方向の位置 [m] 
        * @param convergenceTime [double] 収束時間 [s] 
    */
    std::vector<FrenetPath> generateMaxPosPathCandidates(double latPos, double latSpeed, double latAccel, 
        double lonPos, double lonSpeed, double lonAccel, 
        std::vector<double> targetLatPoses, std::vector<double> targetLatSpeeds, double maxTargetLatSpeed,
        std::vector<double> targetLonPoses, double convergenceTime);

    /**
        * ある速度に到達する直進経路を生成
        * @param latPos [double]   現在の縦方向の位置 [m]
        * @param latSpeed [double] 現在の縦方向の速度 [m/s]
        * @param latAccel double] 現在の縦方向の加速度 [m/s^2] 
        * @param lonPos [double]   現在の横方向の位置 [m] 
        * @param lonSpeed [double] 現在の横方向の速度 [m/s] 
        * @param lonAccel [double] 現在の横方向の加速度 [m/s^2] 
        * @param targetSpeed [double] 目標の速度 [m/s]
        * @param convergenceTime [double] 収束時間 [s] 
    */
    FrenetPath generateSpeedPath(double latPos, double latSpeed, double latAccel, 
        double lonPos, double lonSpeed, double lonAccel, double targetSpeed, double convergenceTime);

};
}
#endif /* ARTERY_FRENETPLANNING_H */