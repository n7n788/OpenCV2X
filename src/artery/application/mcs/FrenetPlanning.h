#ifndef ARTERY_FRENETPLANNING_H
#define ARTERY_FRENETPLANNING_H

#include "FrenetPath.h"
#include <vector>

namespace artery
{

class FrenetPlanning
{
public:
    /*
        * コンストラクタ 
    */
    FrenetPlanning();

    /*
        * 自由走行の経路の候補を生成 
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
    std::vector<FrenetPath> generateFreePathCandidates(double latPos, double latSpeed, double latAccel, 
        double lonPos, double lonSpeed, double lonAccel, 
        double maxTargetLatSpeed, std::vector<double> targetLonPoses, double convergenceTime);
    
    /*
        * 追従の経路の候補を生成 
        * @param latPos [double]   現在の縦方向の位置 [m] 
        * @param latSpeed [double] 現在の縦方向の速度 [m/s]
        * @param latAccel double] 現在の縦方向の加速度 [m/s^2] 
        * @param lonPos [double]   現在の横方向の位置 [m] 
        * @param lonSpeed [double] 現在の横方向の速度 [m/s] 
        * @param lonAccel [double] 現在の横方向の加速度 [m/s^2] 
        * @param maxTargetLatPos [double] 最大の目標の位置 [m]
        * @param targetLatSpeed [double]   目標の縦方向の速度 [m/s]
        * @param convergenceTime [double] 収束時間 [s] 
    */
    std::vector<FrenetPath> generateFollowingPathCandidates(double latPos, double latSpeed, double latAccel, 
        double lonPos, double lonSpeed, double lonAccel, 
        double maxTargetLatPos, double targetLatSpeed, double convergenceTime);
    
    /*
        * コストが最小の経路を選択
        @return [FrenetPath] コストが最小の経路
    */ 
    FrenetPath selectMinCostPath(const std::vector<FrenetPath>& pathCandidates);
};
}
#endif /* ARTERY_FRENETPLANNING_H */