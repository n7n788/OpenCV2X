// ManeuverCoordinationService.cc
#include "artery/application/mcs/ManeuverCoordinationService.h"
#include "artery/application/mcs/ManeuverCoordinationMessage.h"
#include "artery/application/mcs/FrenetPath.h"
#include "artery/application/mcs/FrenetPlanning.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/Identity.h"
#include "artery/utility/IdentityRegistry.h"
#include "artery/utility/InitStages.h"
#include "artery/traci/VehicleController.h"

#include <omnetpp/checkandcast.h>
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>

#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/profile.hpp>

#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>

#include <algorithm>
#include <limits>

namespace artery
{

Define_Module(ManeuverCoordinationService)

ManeuverCoordinationService::~ManeuverCoordinationService()
{
    cancelAndDelete(mTrigger);
}

int ManeuverCoordinationService::numInitStages() const
{
    return InitStages::Total;
}

void ManeuverCoordinationService::initialize(int stage)
{
    if (stage == InitStages::Prepare) {
        ItsG5Service::initialize();
        
        // パラメータの読み込み
        mMaxSpeed = par("maxSpeed").doubleValue();
        mLaneWidth = par("laneWidth").doubleValue();
        mVehicleLength = par("vehicleLength").doubleValue();
        mSafetyDistance = par("safetyDistance").doubleValue();
        mConvTime = par("convergenceTime").doubleValue();
        
        // 利用可能なレーン位置の設定
        mNumLanes = par("numLanes").intValue();
        for (int i = 0; i < mNumLanes; i++) {
            mAvailableLanes.push_back(i * mLaneWidth);
        }
        
        // トリガーメッセージの作成
        mTrigger = new omnetpp::cMessage("MCS trigger");
        
        // 車両データプロバイダの取得
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        mLastUpdateTime = omnetpp::simTime().dbl();
        mLastUpdateTime = omnetpp::simTime().dbl();
        mLaneChangeInProgress = false;
        mTargetLane = -1;
        mLaneChangeStartTime = 0.0;
        
        // 可視化の設定
        mEnableVisualization = par("enableVisualization").boolValue();
        if (mEnableVisualization) {
            try {
                MCMWebVisualizer::getInstance().initialize(this, par("visualizationPort").intValue());
                EV_INFO << "MCM visualization enabled on port " << par("visualizationPort").intValue() << std::endl;
            } catch (const std::exception& e) {
                EV_ERROR << "Failed to initialize MCM visualization: " << e.what() << std::endl;
            }
        }
    }
    else if (stage == InitStages::Self) {
        mTraciId = getFacilities().get_const<Identity>().traci; 

        // 初期レーン位置を確認
        double initialLonPos = mVehicleDataProvider->position().y.value();
        int initialLane = static_cast<int>(initialLonPos / mLaneWidth);
        EV_INFO << "Vehicle " << mTraciId << " starting in lane " << initialLane 
                << " at position (" << mVehicleDataProvider->position().x.value() 
                << ", " << initialLonPos << ")\n";
    }
}

void ManeuverCoordinationService::trigger()
{
    using namespace vanetza;
    
    // MCMの生成
    auto mcm = generate();
    
    // BTPリクエストの設定
    btp::DataRequestB req;
    req.destination_port = host_cast<PortNumber>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
    
    // 車両制御：予定経路に沿って滑らかに移動
    if (!mPlannedPaths.empty()) {
        const FrenetPath& plannedPath = mPlannedPaths.back();

        // 車両コントローラーを取得
        auto vehicleController = getFacilities().get_mutable_ptr<traci::VehicleController>();
        if (vehicleController) {
            try {
                // TraCI APIを取得
                auto& api = vehicleController->getLiteAPI();
                
                // 現在のレーン位置を取得
                double currentLonPos = mVehicleDataProvider->position().y.value();
                int currentLane = static_cast<int>((mLaneWidth * mNumLanes -  currentLonPos) / mLaneWidth);

                // std::cout << mNumLanes << " lanes, lane width: " << mLaneWidth << " m" << std::endl; 
                // 目標レーンを取得 (FrenetPathの最終目標位置から計算)
                const auto& lonPoses = plannedPath.getLonTrajectory().getPoses();
                if (!lonPoses.empty()) {
                    double targetLonPos = lonPoses.back();
                    int targetLane = static_cast<int>((mLaneWidth * mNumLanes - targetLonPos) / mLaneWidth);

                    // std::cout << "Vehicle " << mTraciId << " currentLanePos: " << currentLonPos << " m, " << 
                    //              " targetLanePos: " << targetLonPos << " m, mLaneWidth: " << mLaneWidth << " m" << 
                    //              "currentLane: " << currentLane << ", targetLane: " << targetLane << std::endl;
                    // sumoのデフォルトの車線変更を防止するために、現在の車線と目標車線が同じ場合でも車線変更コマンドを送信
                    // 目標レーンへ向かって車線変更コマンドを実行
                    // duration: 車線変更にかかる秒数（mConvTimeを使用）

                        api.vehicle().changeLane(
                            mTraciId,
                            targetLane,
                            mConvTime
                        );
                    // 現在のレーンと目標レーンが異なる場合、ステータスを変更して車線変更を開始
                    if (!mLaneChangeInProgress && currentLane != targetLane) {
                        EV_INFO << "Vehicle " << mTraciId << " changing lane from " 
                                << currentLane << " to " << targetLane 
                                << " over " << mConvTime << " seconds\n";
                        std::cout << "Vehicle " << mTraciId << " changing lane from " 
                                << currentLane << " to " << targetLane 
                                << " over " << mConvTime << " seconds\n";
                        
                        // 車線変更フラグを設定
                        mLaneChangeInProgress = true;
                        mTargetLane = targetLane;
                        mLaneChangeStartTime = omnetpp::simTime().dbl();
                    }
                    
                    // 車線変更の進行状況をチェック
                    if (mLaneChangeInProgress) {
                        double elapsedTime = omnetpp::simTime().dbl() - mLaneChangeStartTime;
                        
                        // 車線変更が完了したかチェック（時間経過または目標レーンに到達）
                        if (elapsedTime >= mConvTime || currentLane == mTargetLane) {
                            mLaneChangeInProgress = false;
                            EV_INFO << "Vehicle " << mTraciId << " lane change completed\n";
                            std::cout << "Vehicle " << mTraciId << " lane change completed\n";
                        }
                    }
                }
                
                // 縦方向の速度を設定（軌跡から適切な速度を計算）
                const auto& latSpeeds = plannedPath.getLatTrajectory().getSpeeds();
                if (!latSpeeds.empty()) {
                    // 目標速度を設定（急激な速度変化を避けるため徐々に調整）
                    int index = 1;
                    if (index < static_cast<int>(latSpeeds.size())) {
                        double targetSpeed = latSpeeds[index];
                        double currentSpeed = mVehicleDataProvider->speed().value();
                        
                        // 現在速度と目標速度の差が大きい場合は、徐々に調整
                        const double MAX_SPEED_CHANGE = 2.0; // 最大速度変化量 [m/s]
                        double speedDiff = targetSpeed - currentSpeed;
                        double adjustedSpeed;
                        
                        if (std::abs(speedDiff) > MAX_SPEED_CHANGE) {
                            adjustedSpeed = currentSpeed + (speedDiff > 0 ? MAX_SPEED_CHANGE : -MAX_SPEED_CHANGE);
                        } else {
                            adjustedSpeed = targetSpeed;
                        }
                        
                        // 速度を設定
                        vehicleController->setSpeed(adjustedSpeed * boost::units::si::meter_per_second);
                        
                        EV_INFO << "Vehicle " << mTraciId << " speed adjusted to " 
                                << adjustedSpeed << " m/s (target: " << targetSpeed << " m/s)\n";
                        // std::cout << "Vehicle " << mTraciId << " speed adjusted to " << adjustedSpeed << " m/s (target: " << targetSpeed << " m/s)\n";
                    }
                }
                
            } catch (const std::exception& e) {
                EV_ERROR << "Failed to control vehicle " << mTraciId << ": " << e.what() << "\n";
            }
        }
        
        // 最終更新時間を記録
        mLastUpdateTime = omnetpp::simTime().dbl();
    }
    
    // MCMの送信
    request(req, mcm);
    EV_INFO << "Vehicle " << mTraciId << " sent MCM at " << omnetpp::simTime() << "s\n";
}
ManeuverCoordinationMessage* ManeuverCoordinationService::generate()
{
    // 初期設定：前回の交渉受け入れリストを保持、候補経路リストを初期化
    std::set<std::string> previousAcceptedIds = mAcceptedIds;
    mAcceptedIds.clear();
    std::vector<FrenetPath> pathCandidates;
    
    // 現在の車両状態の取得
    double latPos = mVehicleDataProvider->position().x.value();
    double latSpeed = mVehicleDataProvider->speed().value();
    double latAccel = mVehicleDataProvider->acceleration().value();
    double lonPos = mVehicleDataProvider->position().y.value();
    double lonSpeed = 0.0; // 横方向の速度は通常0に近い
    double lonAccel = 0.0; // 横方向の加速度も通常0に近い
   
    double maxSpeed = mMaxSpeed;
    auto vehicleController = getFacilities().get_mutable_ptr<traci::VehicleController>();
    if (vehicleController) maxSpeed = vehicleController->getVehicleType().getMaxSpeed().value();
    // std::cout << "traciId: " << mTraciId << ", maxSpeed: " << maxSpeed << " m/s" << std::endl;

    // レーン中央位置を計算
    std::vector<double> laneCenterPositions;
    for (double lanePos : mAvailableLanes) {
        // レーンの中央位置 = レーンの左端 + レーン幅の半分
        double laneCenterPos = lanePos + (mLaneWidth / 2.0);
        laneCenterPositions.push_back(laneCenterPos);
    }
    
    // 全レーンに対して最高速度に達する候補経路を生成（レーン中央位置を使用）
    auto speedPathCandidates = mPlanner.generateMaxSpeedPathCandidates(
        latPos, latSpeed, latAccel,
        lonPos, lonSpeed, lonAccel,
        maxSpeed, laneCenterPositions, mConvTime
    );
    pathCandidates.insert(pathCandidates.end(), speedPathCandidates.begin(), speedPathCandidates.end());
    
    // 前方車両が存在するレーンに対して、目標位置に達する候補経路を生成
    for (double laneCenterPos : laneCenterPositions) {
        double leadingVehiclePos = getLeadingVehiclePosition(laneCenterPos);
    
        // std::cout << mTraciId << ": leadingVehiclePos: " << leadingVehiclePos << " m" << std::endl;

        if (leadingVehiclePos > 0) { // 前方車両が存在する場合
            double leadingVehicleSpeed = getLeadingVehicleSpeed(laneCenterPos);
            // 先行車両の5秒後の予測位置 = 現在位置 + 速度 * 時間
            double predictedLeadingVehiclePos = leadingVehiclePos + (leadingVehicleSpeed * mConvTime);
            // 目標位置 = 先行車両の5秒後の予測位置 - 自車両の長さ - 安全距離
            double maxTargetPos = predictedLeadingVehiclePos - mVehicleLength - mSafetyDistance;

            // std::cout << mTraciId << ": leadingVehicleSpeed: " << leadingVehicleSpeed << " m/s" << std::endl;
            // std::cout << mTraciId << ": targetPos: " << targetPos << " m" << std::endl; 

            auto posPathCandidates = mPlanner.generateMaxPosPathCandidates(
                latPos, latSpeed, latAccel,
                lonPos, lonSpeed, lonAccel,
                maxTargetPos, leadingVehicleSpeed, maxSpeed, { laneCenterPos }, mConvTime
            );
            pathCandidates.insert(pathCandidates.end(), posPathCandidates.begin(), posPathCandidates.end());
        }
    }
    
    // 受信した希望経路との衝突チェックと交渉受け入れ
    for (const auto& entry : mReceivedDesiredPaths) {
        const std::string& senderId = entry.first;
        const FrenetPath& desiredPath = entry.second;
        
        if (!mPreviousPlannedPath.getLatTrajectory().getPoses().empty() && 
            checkCollision(mPreviousPlannedPath, desiredPath)) {
            // 衝突する希望経路がある場合、受け入れ判定
            if (acceptPath(senderId, desiredPath)) {
                mAcceptedIds.insert(senderId);
            }
        }
    }
    
    // 古い交渉受け入れ情報を削除
    for (auto it = previousAcceptedIds.begin(); it != previousAcceptedIds.end(); ) {
        if (mAcceptedIds.find(*it) == mAcceptedIds.end()) {
            it = previousAcceptedIds.erase(it);
        } else {
            ++it;
        }
    }
    mAcceptedIds.insert(previousAcceptedIds.begin(), previousAcceptedIds.end());
    
    // 最適な予定経路と希望経路の選択
    FrenetPath plannedPath;
    FrenetPath desiredPath;
    
    // 予定経路：自分より通行権が高い車両との予定経路に衝突しない経路から選択
    double minPlannedCost = std::numeric_limits<double>::max();
    for (const auto& path : pathCandidates) {
        bool isValid = true;
        
        // 自分より通行権が高い車両との予定経路との衝突チェック
        for (const auto& entry : mReceivedPlannedPaths) {
            const std::string& otherId = entry.first;
            const FrenetPath& otherPath = entry.second;
            
            if (hasPriority(otherId, mTraciId) && checkCollision(path, otherPath)) {
                isValid = false;
                break;
            }
        }
        
        if (isValid && path.getCost() < minPlannedCost) {
            minPlannedCost = path.getCost();
            plannedPath = path;
        }
    }
    
    // 希望経路：交渉受け入れリストに含まれた車両の希望経路と衝突しない経路から選択
    double minDesiredCost = std::numeric_limits<double>::max();
    for (const auto& path : pathCandidates) {
        bool isValid = true;
        
        // 交渉受け入れリストに含まれた車両の希望経路との衝突チェック
        for (const auto& id : mAcceptedIds) {
            const auto& it = mReceivedDesiredPaths.find(id);
            if (it != mReceivedDesiredPaths.end() && checkCollision(path, it->second)) {
                isValid = false;
                break;
            }
        }
        
        if (isValid && path.getCost() < minDesiredCost) {
            minDesiredCost = path.getCost();
            desiredPath = path;
        }
    }
    
    // MCMの作成
    auto mcm = new ManeuverCoordinationMessage();
    mcm->setTraciId(mTraciId);
    mcm->setPlannedPath(plannedPath);
    mcm->setDesiredPath(desiredPath);
    mcm->setLatPos(latPos);
    mcm->setLatSpeed(latSpeed);
    mcm->setLatAccel(latAccel);
    mcm->setLonPos(lonPos);
    mcm->setLonSpeed(lonSpeed);
    mcm->setLonAccel(lonAccel);
    
    // メッセージサイズの設定（実際のサイズを計算する必要がある）
    mcm->setByteLength(200);
    
    // 経路リストの更新
    mPlannedPaths.push_back(plannedPath);
    mDesiredPaths.push_back(desiredPath);
    mPreviousPlannedPath = plannedPath;
    mPreviousDesiredPath = desiredPath;
    
    // 可視化を更新
    if (mEnableVisualization) {
        MCMWebVisualizer::getInstance().setEgoPaths(mTraciId, plannedPath, desiredPath);
        // 候補経路も可視化に渡す
        MCMWebVisualizer::getInstance().setPathCandidates(mTraciId, pathCandidates);
    }
    
    return mcm;
}

void ManeuverCoordinationService::indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket* packet)
{
    auto mcm = omnetpp::check_and_cast<ManeuverCoordinationMessage*>(packet);
    const std::string& senderId = mcm->getTraciId();
    
    EV_INFO << "Vehicle " << mTraciId << " received MCM from " << senderId << " at " << omnetpp::simTime() << "s\n";
    // std::cout << "Vehicle " << mTraciId << " received MCM from " << senderId << " at " << omnetpp::simTime() << "s\n";
    
    // 受信した経路情報の保存
    mReceivedPlannedPaths[senderId] = mcm->getPlannedPath();
    mReceivedDesiredPaths[senderId] = mcm->getDesiredPath();
    
    // 車両位置情報の更新
    mVehiclePoses[senderId] = std::make_pair(mcm->getLatPos(), mcm->getLonPos());
    mVehicleSpeeds[senderId] = std::make_pair(mcm->getLatSpeed(), mcm->getLonSpeed());
    
    // 可視化を更新
    if (mEnableVisualization) {
        MCMWebVisualizer::getInstance().visualizeMCM(mcm);
    }

    delete packet;
}

void ManeuverCoordinationService::handleMessage(omnetpp::cMessage* msg)
{
    if (msg == mTrigger) {
        trigger();
    } else {
        ItsG5Service::handleMessage(msg);
    }
}

bool ManeuverCoordinationService::checkCollision(const FrenetPath& path1, const FrenetPath& path2)
{
    // 経路の衝突判定ロジック
    // 軌道の各時点で位置を比較し、一定の閾値以下なら衝突と判定
    
    const double LAT_COLLISION_THRESHOLD = mVehicleLength / 2 + mSafetyDistance / 4;
    const double LON_COLLISION_THRESHOLD = mLaneWidth / 2;
    
    const auto& lat1 = path1.getLatTrajectory().getPoses();
    const auto& lon1 = path1.getLonTrajectory().getPoses();
    const auto& lat2 = path2.getLatTrajectory().getPoses();
    const auto& lon2 = path2.getLonTrajectory().getPoses();
    
    // 軌道のサイズチェック
    if (lat1.empty() || lon1.empty() || lat2.empty() || lon2.empty()) {
        return false;
    }
    
    // 軌道の長さは同じであるため最小値を使用
    size_t size = std::min({lat1.size(), lon1.size(), lat2.size(), lon2.size()});
    
    // 各時点で距離をチェック
    for (size_t i = 0; i < size; i++) {
        double dlat = lat1[i] - lat2[i];
        double dlon = lon1[i] - lon2[i];
        
        if (abs(dlat) < LAT_COLLISION_THRESHOLD && abs(dlon) < LON_COLLISION_THRESHOLD) {
            return true; // 衝突あり
        }
    }
    
    return false; // 衝突なし
}

bool ManeuverCoordinationService::hasPriority(const std::string& id1, const std::string& id2)
{
    // 優先度判定ロジック（例：IDの数値の大きさで判定）
    try {
        double num1 = std::stod(id1);
        double num2 = std::stod(id2);
        return num1 <= num2; // 数値が小さいが優先
    } catch (const std::exception&) {
        // 数値変換できない場合は文字列比較
        return id1 <= id2;
    }
}

bool ManeuverCoordinationService::acceptPath(const std::string& senderId, const FrenetPath& desiredPath)
{
    // 経路受け入れ判定ロジック
    // とりあえず常に受け入れる
    return true;
}

double ManeuverCoordinationService::getLeadingVehiclePosition(double laneCenterPosition)
{
    // 前方車両の位置を取得する処理
    // ローカル環境モデルから前方車両の情報を取得
    
    double minDistance = std::numeric_limits<double>::max();
    double leadingVehiclePos = -1; // 前方車両なしを示す値
    
    const double LANE_THRESHOLD = mLaneWidth / 2; // レーン判定の閾値
    
    for (const auto& entry : mVehiclePoses) {
        const std::string& vehicleId = entry.first;
        const auto& pos = entry.second;
        
        // 自車両は除外
        if (vehicleId == mTraciId) continue;
        
        // 同一レーン上の車両かチェック（レーン中央位置との距離で判定）
        if (std::abs(pos.second - laneCenterPosition) <= LANE_THRESHOLD) {
            // 前方にある車両かチェック
            double distance = pos.first - mVehicleDataProvider->position().x.value();
            if (distance > 0 && distance < minDistance) {
                minDistance = distance;
                leadingVehiclePos = pos.first;
            }
        }
    }
    
    return leadingVehiclePos;
}

double ManeuverCoordinationService::getLeadingVehicleSpeed(double laneCenterPosition)
{
    // 前方車両の速度を取得する処理
    
    double minDistance = std::numeric_limits<double>::max();
    double leadingVehicleSpeed = DBL_MAX; // デフォルトは最大速度
    
    const double LANE_THRESHOLD = mLaneWidth / 2;
    
    for (const auto& entry : mVehiclePoses) {
        const std::string& vehicleId = entry.first;
        const auto& pos = entry.second;
        
        // 自車両は除外
        if (vehicleId == mTraciId) continue;
        
        // 同一レーン上の車両かチェック（レーン中央位置との距離で判定）
        if (std::abs(pos.second - laneCenterPosition) <= LANE_THRESHOLD) {
            // 前方にある車両かチェック
            double distance = pos.first - mVehicleDataProvider->position().x.value();
            if (distance > 0 && distance < minDistance) {
                minDistance = distance;
                // 速度情報を取得
                const auto& speedIt = mVehicleSpeeds.find(vehicleId);
                if (speedIt != mVehicleSpeeds.end()) {
                    leadingVehicleSpeed = speedIt->second.first;
                }
            }
        }
    }
    
    return leadingVehicleSpeed;
}

void ManeuverCoordinationService::finish()
{
    ItsG5Service::finish();
    
    if (mEnableVisualization) {
        MCMWebVisualizer::getInstance().close();
    }
}

} // namespace artery