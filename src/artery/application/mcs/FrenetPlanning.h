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
        * @param mLonPos [double]   現在の縦方向の位置 
        * @param mLonSpeed [double] 現在の縦方向の速度
        * @param mLonAccel [double] 現在の縦方向の加速度 
        * @param mLatPos [double]   現在の横方向の位置 
        * @param mLatSpeed [double] 現在の横方向の速度 
        * @param mLatAccel [double] 現在の横方向の加速度 
        * @param mTargetLatMaxSpeed [double]    目標の縦方向の速度の最大値
        * @param mTargetlatPoses [vector<double>]  目標の横方向の位置 
        * @param mConvergenceTime [double] 収束時間 
    */
    std::vector<FrenetPath> generateFreePathCandidates(double mLonPos, double mLonSpeed, double mLonAccel, 
        double mLatPos, double mLatSpeed, double mLatAccel, 
        double mTargetLatMaxSpeed, std::vector<double> mTargetLatPoses, double mConvergenceTime);
};
}
#endif /* ARTERY_FRENETPLANNING_H */