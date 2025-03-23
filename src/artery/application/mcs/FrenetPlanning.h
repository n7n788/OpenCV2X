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
        * 縦方向の最高速度, 横方向の位置集合に到達するような、経路の候補を生成 
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
    
    /*
        * 縦方向の最長位置, 横方向の位置集合に到達するような、経路の候補を生成 
        *  目標速度は固定
        * @param latPos [double]   現在の縦方向の位置 [m] 
        * @param latSpeed [double] 現在の縦方向の速度 [m/s]
        * @param latAccel double] 現在の縦方向の加速度 [m/s^2] 
        * @param lonPos [double]   現在の横方向の位置 [m] 
        * @param lonSpeed [double] 現在の横方向の速度 [m/s] 
        * @param lonAccel [double] 現在の横方向の加速度 [m/s^2] 
        * @param maxTargetLatPos [double] 最大の目標の位置 [m]
        * @param targetLatSpeed [double]   目標の縦方向の速度 [m/s]
        * @param targetLonPoses [vector<double>]  目標の横方向の位置 [m] 
        * @param convergenceTime [double] 収束時間 [s] 
    */
    std::vector<FrenetPath> generateMaxPosPathCandidates(double latPos, double latSpeed, double latAccel, 
        double lonPos, double lonSpeed, double lonAccel, 
        double maxTargetLatPos, double targetLatSpeed, std::vector<double> targetLonPoses, double convergenceTime);
    
    /*
        * コストが最小の経路を選択
        @return [FrenetPath] コストが最小の経路
    */ 
    FrenetPath selectMinCostPath(const std::vector<FrenetPath>& pathCandidates);
};
}
#endif /* ARTERY_FRENETPLANNING_H */