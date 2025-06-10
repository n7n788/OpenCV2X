#ifndef ARTERY_PATH_PLANNER_H
#define ARTERY_PATH_PLANNER_H

#include "Path.h"
#include "PathGenerator.h"
#include "CollisionDetector.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"
#include <vector>
#include <map>
#include <string>
#include <set>

namespace artery {

/**
 * @brief 車両の経路計画を担当するクラス
 * 
 * ManeuverCoordinationServiceから経路生成・選択のロジックを分離し、
 * 単一責任の原則に従って経路計画のみを担当する。
 * 衝突判定はCollisionDetectorに委譲する。
 */
class PathPlanner {
public:
    /**
     * @brief コンストラクタ
     * @param generator 経路生成エンジンへの参照
     * @param detector 衝突判定器への参照
     * @param laneWidth レーン幅 [m]
     * @param numLanes レーン数
     * @param vehicleLength 車両長 [m]
     * @param safetySecond 安全時間 [s]
     * @param convTime 収束時間 [s]
     */
    PathPlanner(PathGenerator& generator, const CollisionDetector& detector,
                double laneWidth, int numLanes, 
                double vehicleLength, double safetySecond, double convTime);
    
    /**
     * @brief 候補経路を生成する
     * @param vehicleDataProvider 車両状態プロバイダ
     * @param vehicleController 車両コントローラー
     * @param vehiclePoses 他車両位置情報
     * @param vehicleSpeeds 他車両速度情報
     * @param myTraciId 自車両ID
     * @param lastLaneChangeTime 最後の車線変更時刻
     * @param laneChangeInterval 車線変更インターバル
     * @return 候補経路リスト
     */
    std::vector<Path> generateCandidatePaths(
        const VehicleDataProvider* vehicleDataProvider,
        traci::VehicleController* vehicleController,
        const std::map<std::string, std::pair<double, double>>& vehiclePoses,
        const std::map<std::string, std::pair<double, double>>& vehicleSpeeds,
        const std::string& myTraciId,
        double lastLaneChangeTime,
        double laneChangeInterval
    );
    
    /**
     * @brief 最適な予定経路を選択する
     * @param candidates 候補経路リスト
     * @param receivedPlannedPaths 受信予定経路
     * @param receivedDesiredPaths 受信希望経路
     * @param acceptedIds 交渉受け入れリスト
     * @param myTraciId 自車両ID
     * @param vehiclePoses 他車両位置情報
     * @param vehicleDataProvider 車両状態プロバイダ
     * @return 選択された予定経路
     */
    Path selectPlannedPath(
        const std::vector<Path>& candidates,
        const std::map<std::string, Path>& receivedPlannedPaths,
        const std::map<std::string, Path>& receivedDesiredPaths,
        const std::set<std::string>& acceptedIds,
        const std::string& myTraciId,
        const std::map<std::string, std::pair<double, double>>& vehiclePoses,
        const VehicleDataProvider* vehicleDataProvider
    );
    
    /**
     * @brief 最適な希望経路を選択する
     * @param candidates 候補経路リスト
     * @param receivedPlannedPaths 受信予定経路
     * @param receivedDesiredPaths 受信希望経路
     * @param acceptedIds 交渉受け入れリスト
     * @param plannedPath 予定経路
     * @param costThreshold コスト閾値
     * @param myTraciId 自車両ID
     * @return 選択された希望経路
     */
    Path selectDesiredPath(
        const std::vector<Path>& candidates,
        const std::map<std::string, Path>& receivedPlannedPaths,
        const std::map<std::string, Path>& receivedDesiredPaths,
        const std::set<std::string>& acceptedIds,
        const Path& plannedPath,
        double costThreshold,
        const std::string& myTraciId
    );

private:
    PathGenerator& mGenerator;        ///< 経路生成エンジン
    const CollisionDetector& mCollisionDetector; ///< 衝突判定器への参照
    double mLaneWidth;               ///< レーン幅 [m]
    int mNumLanes;                   ///< レーン数
    std::vector<double> mCenterLanes; ///< レーン中央位置
    double mVehicleLength;           ///< 車両長 [m]
    double mSafetySecond;            ///< 安全時間 [s]
    double mConvTime;                ///< 収束時間 [s]
    
    /**
     * @brief 利用可能レーンを取得
     * @param vehicleDataProvider 車両状態プロバイダ
     * @param lastLaneChangeTime 最後の車線変更時刻
     * @param laneChangeInterval 車線変更インターバル 
     * @return 利用可能レーン位置リスト 
     * */
    std::vector<double> getAvailableLanes( const VehicleDataProvider* vehicleDataProvider, double lastLaneChangeTime, double laneChangeInterval);

    /**
     * @brief 前方車両位置を取得
     * @param lanePosition レーン位置
     * @param vehiclePoses 他車両位置情報
     * @param vehicleDataProvider 車両状態プロバイダ
     * @param myTraciId 自車両ID
     * @return 前方車両位置（存在しない場合-1）
     */
    double getLeadingVehiclePosition(double lanePosition,
        const std::map<std::string, std::pair<double, double>>& vehiclePoses,
        const VehicleDataProvider* vehicleDataProvider,
        const std::string& myTraciId);
    
    /**
     * @brief 前方車両速度を取得
     * @param lanePosition レーン位置
     * @param vehiclePoses 他車両位置情報
     * @param vehicleSpeeds 他車両速度情報
     * @param vehicleDataProvider 車両状態プロバイダ
     * @param myTraciId 自車両ID
     * @return 前方車両速度
     */
    double getLeadingVehicleSpeed(double lanePosition,
        const std::map<std::string, std::pair<double, double>>& vehiclePoses,
        const std::map<std::string, std::pair<double, double>>& vehicleSpeeds,
        const VehicleDataProvider* vehicleDataProvider,
        const std::string& myTraciId);
        
    /**
     * @brief 2つの経路が衝突するかを判定
     * @param path1 経路1
     * @param path2 経路2
     * @return 衝突する場合true
     */
    bool checkCollision(const Path& path1, const Path& path2);
    
    /**
     * @brief 他車両が自車両より優先度が高いかを判定
     * @param senderId 他車両ID
     * @param path 自車両の経路
     * @param vehiclePoses 他車両位置情報
     * @param vehicleDataProvider 自車両状態プロバイダ
     * @return 他車両の方が優先度が高い場合true
     */
    bool hasPriority(const std::string& senderId, const Path& path,
        const std::map<std::string, std::pair<double, double>>& vehiclePoses,
        const VehicleDataProvider* vehicleDataProvider);
};

} // namespace artery

#endif
