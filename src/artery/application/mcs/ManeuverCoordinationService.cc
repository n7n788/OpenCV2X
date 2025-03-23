// ManeuverCoordinationService.cc
#include "artery/application/mcs/ManeuverCoordinationService.h"
#include "artery/application/mcs/ManeuverCoordinationMessage.h"
#include "artery/application/mcs/FrenetPath.h"
#include "artery/application/mcs/FrenetPlanning.h"
#include "artery/application/VehicleDataProvider.h"
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
        int numLanes = par("numLanes");
        for (int i = 0; i < numLanes; i++) {
            mAvailableLanes.push_back(i * mLaneWidth);
        }
        
        // トリガーメッセージの作成
        mTrigger = new omnetpp::cMessage("MCS trigger");
        
        // 車両データプロバイダの取得
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        
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
    
    // 車両制御：予定経路に基づいて速度と方向を調整
    if (!mPlannedPaths.empty()) {
        const FrenetPath& plannedPath = mPlannedPaths.back();
        // 速度と方向の調整コードを実装（TraCIインターフェースを使用）
        // ...
    }
    
    // MCMの送信
    request(req, mcm);
    EV_INFO << "Vehicle " << mTraciId << " sent MCM at " << omnetpp::simTime() << "s\n";

    std::cout << "Vehicle " << mTraciId << " sent MCM at " << omnetpp::simTime() << "s\n";
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
    
    // 全レーンに対して最高速度に達する候補経路を生成
    auto speedPathCandidates = mPlanner.generateMaxSpeedPathCandidates(
        latPos, latSpeed, latAccel,
        lonPos, lonSpeed, lonAccel,
        mMaxSpeed, mAvailableLanes, mConvTime
    );
    pathCandidates.insert(pathCandidates.end(), speedPathCandidates.begin(), speedPathCandidates.end());
    
    // 前方車両が存在するレーンに対して、目標位置に達する候補経路を生成
    for (double lanePos : mAvailableLanes) {
        double leadingVehiclePos = getLeadingVehiclePosition(lanePos);
        if (leadingVehiclePos > 0) { // 前方車両が存在する場合
            double leadingVehicleSpeed = getLeadingVehicleSpeed(lanePos);
            double targetPos = leadingVehiclePos - mVehicleLength - mSafetyDistance;
            
            auto posPathCandidates = mPlanner.generateMaxPosPathCandidates(
                latPos, latSpeed, latAccel,
                lonPos, lonSpeed, lonAccel,
                targetPos, leadingVehicleSpeed, { lanePos }, mConvTime
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
    }
    
    return mcm;
}

void ManeuverCoordinationService::indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket* packet)
{
    auto mcm = omnetpp::check_and_cast<ManeuverCoordinationMessage*>(packet);
    const std::string& senderId = mcm->getTraciId();
    
    EV_INFO << "Vehicle " << mTraciId << " received MCM from " << senderId << " at " << omnetpp::simTime() << "s\n";
    std::cout << "Vehicle " << mTraciId << " received MCM from " << senderId << " at " << omnetpp::simTime() << "s\n";
    
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
    
    const double COLLISION_THRESHOLD = mVehicleLength / 2 + mSafetyDistance / 4;
    
    const auto& lat1 = path1.getLatTrajectory().getPoses();
    const auto& lon1 = path1.getLonTrajectory().getPoses();
    const auto& lat2 = path2.getLatTrajectory().getPoses();
    const auto& lon2 = path2.getLonTrajectory().getPoses();
    
    // 軌道のサイズチェック
    if (lat1.empty() || lon1.empty() || lat2.empty() || lon2.empty()) {
        return false;
    }
    
    // 軌道の長さは同じであるiき
    size_t size = std::min({lat1.size(), lon1.size(), lat2.size(), lon2.size()});
    
    // 各時点で距離をチェック
    for (size_t i = 0; i < size; i++) {
        double dx = lat1[i] - lat2[i];
        double dy = lon1[i] - lon2[i];
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < COLLISION_THRESHOLD) {
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
        return num1 > num2; // 数値が大きい方が優先
    } catch (const std::exception&) {
        // 数値変換できない場合は文字列比較
        return id1 > id2;
    }
}

bool ManeuverCoordinationService::acceptPath(const std::string& senderId, const FrenetPath& desiredPath)
{
    // 経路受け入れ判定ロジック
    // 例：優先度の高い車両の経路は受け入れる
    if (hasPriority(senderId, mTraciId)) {
        return true;
    }
    
    // 他の条件（例：衝突の深刻度、経路のコストなど）による判定を追加可能
    
    return false; // デフォルトでは拒否
}

double ManeuverCoordinationService::getLeadingVehiclePosition(double lanePosition)
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
        
        // 同一レーン上の車両かチェック
        if (std::abs(pos.second - lanePosition) <= LANE_THRESHOLD) {
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

double ManeuverCoordinationService::getLeadingVehicleSpeed(double lanePosition)
{
    // 前方車両の速度を取得する処理
    
    double minDistance = std::numeric_limits<double>::max();
    double leadingVehicleSpeed = mMaxSpeed; // デフォルトは最大速度
    
    const double LANE_THRESHOLD = mLaneWidth / 2;
    
    for (const auto& entry : mVehiclePoses) {
        const std::string& vehicleId = entry.first;
        const auto& pos = entry.second;
        
        // 自車両は除外
        if (vehicleId == mTraciId) continue;
        
        // 同一レーン上の車両かチェック
        if (std::abs(pos.second - lanePosition) <= LANE_THRESHOLD) {
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