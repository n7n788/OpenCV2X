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
        mLaneWidth = par("laneWidth").doubleValue();
        mVehicleLength = par("vehicleLength").doubleValue();
        mConvTime = par("convergenceTime").doubleValue();
        
        // 利用可能なレーン位置の設定
        mNumLanes = par("numLanes").intValue();
        for (int i = 0; i < mNumLanes; i++) {
            mCenterLanes.push_back(i * mLaneWidth + mLaneWidth / 2.0);
        }
        
        // トリガーメッセージの作成
        mTrigger = new omnetpp::cMessage("MCS trigger");
        
        // 車両データプロバイダの取得
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        // 車両のコントローラを取得
        mVehicleController = getFacilities().get_mutable_ptr<traci::VehicleController>();
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
    if (!mPlannedPaths.empty() && !isObstacle(mTraciId)) {
        const FrenetPath& plannedPath = mPlannedPaths.back();
        // TraCI APIを取得
        auto& api = mVehicleController->getLiteAPI();
        
        // 現在のレーン位置を取得
        double currentLonPos = mVehicleDataProvider->position().y.value();
        int currentLane = static_cast<int>((mLaneWidth * mNumLanes -  currentLonPos) / mLaneWidth);

        // 目標レーンを取得 (FrenetPathの最終目標位置から計算)
        const auto& lonPoses = plannedPath.getLonTrajectory().getPoses();
        if (!lonPoses.empty()) {
            double targetLonPos = lonPoses.back();
            int targetLane = static_cast<int>((mLaneWidth * mNumLanes - targetLonPos) / mLaneWidth);

            // 現在のレーンと目標レーンが異なる場合、ステータスを変更して車線変更を開始
            if (!mLaneChangeInProgress && currentLane != targetLane) {
                api.vehicle().changeLane(
                    mTraciId,
                    targetLane,
                    mConvTime
                );
                std::cout << "Vehicle " << mTraciId << " changing lane from " 
                        << currentLane << " to " << targetLane 
                        << " over " << mConvTime << " seconds\n";
                
                // 車線変更フラグを設定
                mLaneChangeInProgress = true;
                mTargetLane = targetLane;
                mLaneChangeStartTime = omnetpp::simTime().dbl();
            }
            
            // // 車線変更の進行状況をチェック
            if (mLaneChangeInProgress) {
                double elapsedTime = omnetpp::simTime().dbl() - mLaneChangeStartTime;
                
                // 車線変更が完了したかチェック（時間経過または目標レーンに到達）
                if (elapsedTime >= mConvTime || currentLane == mTargetLane) {
                    mLaneChangeInProgress = false;
                    mLastLaneChangeTime = omnetpp::simTime().dbl();
                    std::cout << "Vehicle " << mTraciId << " completed lane change to " 
                              << mTargetLane << " time: " << mLastLaneChangeTime << "\n";
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
                double adjustedSpeed;

                // 急激な速度変化を避けるため、現在の速度と目標速度の差を制限
                // double maxSpeedDiff = 0.3; // 最大速度差
                // if (currentSpeed > targetSpeed) {
                //     // 減速する場合
                //     adjustedSpeed = currentSpeed - std::min(currentSpeed - targetSpeed, maxSpeedDiff);
                // } else {
                //     // 加速する場合
                //     adjustedSpeed = currentSpeed + std::min(targetSpeed - currentSpeed, maxSpeedDiff);  
                // }

                // 速度を設定
                mVehicleController->setSpeed(targetSpeed * boost::units::si::meter_per_second);
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
    // 現在の車両状態の取得
    double latPos = mVehicleDataProvider->position().x.value();
    double latSpeed = mVehicleDataProvider->speed().value();
    double latAccel = mVehicleDataProvider->acceleration().value();
    double lonPos = mVehicleDataProvider->position().y.value();
    double lonSpeed = 0.0; // 横方向の速度は通常0に近い
    double lonAccel = 0.0; // 横方向の加速度も通常0に近い
    double maxSpeed = mVehicleController->getMaxSpeed().value();

    // 候補経路を計算
    std::vector<FrenetPath> pathCandidates;
    // 到達可能なレーン中央位置を計算
    std::vector<double> laneCenterPositions;

    // 車線変更後にインターバル経過した場合、車線変更を許して候補経路を生成する 
    if (mLastLaneChangeTime < mLaneChangeInterval || omnetpp::simTime().dbl() - mLastLaneChangeTime > mLaneChangeInterval) {
        for (double l : mCenterLanes) {
            if (abs(l - lonPos) < mLaneWidth / 2.0 || 
                abs(l - lonPos - mLaneWidth) < mLaneWidth / 2.0 || 
                abs(l - lonPos + mLaneWidth) < mLaneWidth / 2.0) {
                // 車線変更可能な場合、レーン中央位置を追加
                laneCenterPositions.push_back(l);
            }
        }
    } else {
        // 車線変更不可
        laneCenterPositions.push_back(lonPos); 
    }
    
    // 全レーンに対して、最高速度に達する候補経路を生成（レーン中央位置を使用）
    auto speedPathCandidates = mPlanner.generateMaxSpeedPathCandidates(
        latPos, latSpeed, latAccel,
        lonPos, lonSpeed, lonAccel,
        maxSpeed, laneCenterPositions, mConvTime
    );
    pathCandidates.insert(pathCandidates.end(), speedPathCandidates.begin(), speedPathCandidates.end());

    // 全レーンに対して、前方車両の位置と速度を取得 
    std::vector<double> leadingLatPoses;
    std::vector<double> leadingLatSpeeds;
    std::vector<double> leadingLonPoses;
    for (double laneCenterPos : laneCenterPositions) {
        double leadingVehiclePos = getLeadingVehiclePosition(laneCenterPos);
        if (leadingVehiclePos > 0) { // 前方車両が存在する場合
            double leadingVehicleSpeed = getLeadingVehicleSpeed(laneCenterPos);
            // 先行車両の5秒後の予測位置 = 現在位置 + 速度 * 時間
            double predictedLeadingVehiclePos = leadingVehiclePos + (leadingVehicleSpeed * mConvTime);
            // 目標位置 = 先行車両の5秒後の予測位置 - 自車両の長さ - 安全距離
            double maxTargetPos = predictedLeadingVehiclePos - mVehicleLength - latSpeed * mSafetySecond;

            leadingLatPoses.push_back(leadingVehiclePos);
            leadingLatSpeeds.push_back(leadingVehicleSpeed);
            leadingLonPoses.push_back(laneCenterPos);
        }
    }

    // 前方車両が存在するレーンに対して、目標位置に達する候補経路を生成
    auto posPathCandidates = mPlanner.generateMaxPosPathCandidates(
        latPos, latSpeed, latAccel,
        lonPos, lonSpeed, lonAccel,
        leadingLatPoses, leadingLatSpeeds, maxSpeed, leadingLonPoses, mConvTime
    );
    pathCandidates.insert(pathCandidates.end(), posPathCandidates.begin(), posPathCandidates.end());


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
    
    // 最適な予定経路と希望経路の選択
    FrenetPath plannedPath = FrenetPath();
    FrenetPath desiredPath = FrenetPath();
    
    // 予定経路: 以下の全てと衝突しない経路の中でコストが最小の経路を選択
    // 自分より通行権が高い車両の予定経路
    // 交渉受け入れリストに含まれた車両の希望経路
    // 障害物
    double minPlannedCost = std::numeric_limits<double>::max();
    for (const auto& path : pathCandidates) {
        bool isValid = true;
        
        // 自分より通行権が高い車両との予定経路との衝突チェック
        for (const auto& entry : mReceivedPlannedPaths) {
            const std::string& otherId = entry.first;
            const FrenetPath& otherPath = entry.second;
            
            if (hasPriority(otherId, path) && checkCollision(path, otherPath)) {
                isValid = false;
                break;
            }
        }
        
        // 交渉受け入れリストに含まれた車両の希望経路との衝突チェック
        for (const auto& id : mAcceptedIds) {
            const auto& it = mReceivedDesiredPaths.find(id);
            if (it != mReceivedDesiredPaths.end() && checkCollision(path, it->second)) {
                isValid = false;
                break;
            }
        }

        if (isValid && path.getCost() < minPlannedCost) {
            minPlannedCost = path.getCost();
            plannedPath = path;
        }
    }

    // 希望経路: 以下の全てと衝突しない経路の中でコストが最小の経路を選択
    // 交渉受け入れリストに含まれた車両の希望経路
    // 障害物
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

        // 障害物との衝突チェック
        for (const auto& entry : mReceivedPlannedPaths) {
            const std::string& otherId = entry.first;
            const FrenetPath& otherPath = entry.second;
            
            if (isObstacle(otherId) && checkCollision(path, otherPath)) {
                isValid = false;
                break;
            }
        }
        
        if (isValid && path.getCost() < minDesiredCost) {
            minDesiredCost = path.getCost();
            desiredPath = path;
        }
    }

    // 希望経路と予定経路のコスト差分が閾値未満である場合、希望経路を送信しない
    if (plannedPath.getCost() - desiredPath.getCost() < mDesiredCostThreshold) {
        desiredPath = FrenetPath(); // 希望経路を空にする
    }

    // 予定経路がない場合、等速で移動する経路を使用 
    // TO DO 修正したい
    if (plannedPath.getCost() < 0) {
        plannedPath = mPlanner.generateSpeedPath(latPos, latSpeed, latAccel, lonPos, lonSpeed, lonAccel, latSpeed, mConvTime); 
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
    mLastGenerateMcmTime = omnetpp::simTime().dbl();
    
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

    const double LAT_COLLISION_THRESHOLD = mVehicleLength / 2;
    const double LON_COLLISION_THRESHOLD = mLaneWidth / 2;
    
    const auto& lat1 = path1.getLatTrajectory().getPoses();
    const auto& lat1Speeds = path1.getLatTrajectory().getSpeeds();
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
        
        if (abs(dlat) < LAT_COLLISION_THRESHOLD + lat1Speeds[i] * mSafetySecond && abs(dlon) < LON_COLLISION_THRESHOLD) {
            return true; // 衝突あり
        }
    }
    
    return false; // 衝突なし
}

bool ManeuverCoordinationService::hasPriority(const std::string& senderId, const FrenetPath& path)
{
    // 優先度判定ロジック
    // 障害物は最も優先度が高い
    if (isObstacle(mTraciId)) return false;
    if (isObstacle(senderId)) return true;

    auto it = mVehiclePoses.find(senderId);
    if (it == mVehiclePoses.end()) return false; // senderIdの車両が存在しない場合は優先度が低い

    const auto& pos = it->second;
    double latPos = pos.first;
    double lonPos = pos.second;
    double myLatPos = mVehicleDataProvider->position().x.value();
    double myLonPos = mVehicleDataProvider->position().y.value();
    double myLastLonPos = path.getLonTrajectory().getPoses().back(); 

    // 自車両が車線変更しようとしており、senderIdの車両が変更先の車線にいる場合、senderIdの方が優先度が高い
    if (std::abs(myLastLonPos - myLonPos) > mLaneWidth / 2) {
        if (std::abs(lonPos - myLastLonPos) < mLaneWidth / 2) return true;
        if (myLonPos < lonPos && lonPos < myLastLonPos) return true;
        if (myLonPos > lonPos && lonPos > myLastLonPos) return true;
    }
    
    // 自車両が車線変更せず、senderIdと車線が同じで、sendeIdが先行車両の場合、senderIdの方が優先度が高い
    if (std::abs(myLastLonPos - myLonPos) < mLaneWidth / 2 && std::abs(lonPos - myLastLonPos) < mLaneWidth / 2) {
        // 自車両の位置とsenderIdの車両の位置を比較
        if (latPos > myLatPos) {
            return true; // senderIdの方が優先度が高い
        }
    }

    // if (mTraciId == "0.0") std::cout << "Vehicle " << mTraciId << " has priority over " << senderId << std::endl;

    return false;
} 

bool ManeuverCoordinationService::acceptPath(const std::string& senderId, const FrenetPath& desiredPath)
{
    // 経路受け入れ判定ロジック
    // 30%の確率で受け入れる
    // return rand() % 10 < 3;
    // std::cout << "Vehicle " << mTraciId << " accepted path from " << senderId << std::endl;
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
            double mySpeed = mVehicleDataProvider->speed().value();
            double safetyMargin = mVehicleLength / 2 + mSafetySecond * mySpeed;
            double distance = pos.first - mVehicleDataProvider->position().x.value();
            if (distance > safetyMargin && distance < minDistance) {
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

bool ManeuverCoordinationService::isObstacle(const std::string& traciId) const
{
    // 障害物判定ロジック
    // 例えば、IDが特定のパターンに一致する場合は障害物とみなす
    return traciId.compare(0, mObstacle.length(), mObstacle) == 0;
}

void ManeuverCoordinationService::finish()
{
    ItsG5Service::finish();
    
    if (mEnableVisualization) {
        MCMWebVisualizer::getInstance().close();
    }
}

} // namespace artery