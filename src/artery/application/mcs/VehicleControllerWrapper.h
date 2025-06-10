#ifndef ARTERY_VEHICLE_CONTROLLER_WRAPPER_H
#define ARTERY_VEHICLE_CONTROLLER_WRAPPER_H

#include "Path.h"
#include "artery/traci/VehicleController.h"
#include "artery/application/VehicleDataProvider.h"

namespace artery {

/**
 * @brief 車両制御の詳細ロジックをラップするクラス
 * 
 * 計画された経路に基づいて実際の車両制御を実行し、
 * 車線変更と速度制御の状態を管理する。
 */
class VehicleControllerWrapper {
public:
    /**
     * @brief コンストラクタ
     * @param controller 車両コントローラー
     * @param dataProvider 車両状態プロバイダ
     * @param laneWidth レーン幅 [m]
     * @param numLanes レーン数
     * @param convTime 収束時間 [s]
     */
    VehicleControllerWrapper(traci::VehicleController* controller,
                           const VehicleDataProvider* dataProvider,
                           double laneWidth, int numLanes, double convTime);
    
    /**
     * @brief 計画された経路を実行する
     * @param plannedPath 実行する経路
     * @param traciId 車両ID
     */
    void executePath(const Path& plannedPath, const std::string& traciId);
    
    /**
     * @brief 車線変更が進行中かを取得
     * @return 進行中の場合true
     */
    bool isLaneChangeInProgress() const { return mLaneChangeInProgress; }
    
    /**
     * @brief 最後の車線変更時刻を取得
     * @return 最後の車線変更時刻 [s]
     */
    double getLastLaneChangeTime() const { return mLastLaneChangeTime; }

private:
    traci::VehicleController* mController;      ///< 車両コントローラー
    const VehicleDataProvider* mDataProvider;  ///< 車両状態プロバイダ
    double mLaneWidth;                         ///< レーン幅 [m]
    int mNumLanes;                             ///< レーン数
    double mConvTime;                          ///< 収束時間 [s]
    
    bool mLaneChangeInProgress = false;        ///< 車線変更進行フラグ
    int mTargetLane = -1;                      ///< 目標レーン番号
    double mLaneChangeStartTime = 0.0;         ///< 車線変更開始時刻 [s]
    double mLastLaneChangeTime = 0.0;          ///< 最後の車線変更時刻 [s]
    
    /**
     * @brief 車線変更制御を処理
     * @param plannedPath 計画経路
     * @param traciId 車両ID
     */
    void handleLaneChange(const Path& plannedPath, const std::string& traciId);
    
    /**
     * @brief 速度制御を処理
     * @param plannedPath 計画経路
     */
    void handleSpeedControl(const Path& plannedPath);
};

} // namespace artery

#endif
