#ifndef ARTERY_COLLISION_DETECTOR_H
#define ARTERY_COLLISION_DETECTOR_H

#include "Path.h"
#include "artery/application/VehicleDataProvider.h"
#include <string>
#include <map>

namespace artery {

/**
 * @brief 経路間の衝突判定と優先度判定を担当するクラス
 * 
 * 車両間の衝突回避のための判定ロジックを集約し、
 * 安全な協調走行を実現する。
 */
class CollisionDetector {
public:
    /**
     * @brief コンストラクタ
     * @param vehicleLength 車両長 [m]
     * @param laneWidth レーン幅 [m]
     * @param safetySecond 安全時間 [s]
     */
    CollisionDetector(double vehicleLength, double laneWidth, double safetySecond);
    
    /**
     * @brief 2つの経路が衝突するかを判定
     * @param path1 経路1
     * @param path2 経路2
     * @return 衝突する場合true
     */
    bool checkCollision(const Path& path1, const Path& path2) const;
    
    /**
     * @brief 他車両が自車両より優先度が高いかを判定
     * @param senderId 他車両ID
     * @param myPath 自車両の経路
     * @param vehiclePoses 他車両位置情報
     * @param vehicleDataProvider 自車両状態プロバイダ
     * @return 他車両の方が優先度が高い場合true
     */
    bool hasPriority(const std::string& senderId, const Path& myPath,
                    const std::map<std::string, std::pair<double, double>>& vehiclePoses,
                    const VehicleDataProvider* vehicleDataProvider) const;

    /**
     * @brief 車両が障害物かどうかを判定
     * @param traciId 車両ID
     * @return 障害物の場合true
     */
    bool isObstacle(const std::string& traciId) const;
private:
    double mVehicleLength; ///< 車両長 [m]
    double mLaneWidth;     ///< レーン幅 [m]
    double mSafetySecond;  ///< 安全時間 [s]
    
};

} // namespace artery

#endif
