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
    if (!mPlannedPaths.empty() && !mPlannedPaths.back().getLatTrajectory().getPoses().empty()) {
        const FrenetPath& plannedPath = mPlannedPaths.back();
        
        // TraCIを使って車両を制御
        auto vehicleController = getFacilities().get_mutable_ptr<traci::VehicleController>();
        if (vehicleController) {
            // 現在の車両位置を取得
            double currentLatPos = mVehicleDataProvider->position().x.value();
            double currentLonPos = mVehicleDataProvider->position().y.value();
            
            // 予定経路から次の目標点を決定（例：0.5秒後の位置）
            const auto& latTrajectory = plannedPath.getLatTrajectory();
            const auto& lonTrajectory = plannedPath.getLonTrajectory();
            
            // 次の目標点のインデックスを計算（サンプリング間隔は0.1秒）
            int nextPointIndex = std::min(5, static_cast<int>(latTrajectory.getPoses().size()) - 1); // 0.5秒後

            // for (int i = 0; i < lonTrajectory.getPoses().size(); i++) {
            //     std::cout << "lonTrajectory.getPoses()[" << i << "]: " << lonTrajectory.getPoses()[i] << std::endl;
            // }
            
            if (nextPointIndex >= 0) {
                // 次の目標点の位置と速度
                double targetLatPos = latTrajectory.getPoses()[nextPointIndex];
                double targetLonPos = lonTrajectory.getPoses().back();
                double targetLatSpeed = latTrajectory.getSpeeds()[nextPointIndex];
                
                EV_INFO << "Vehicle " << mTraciId << " target: pos=(" << targetLatPos << "," << targetLonPos
                       << "), speed=" << targetLatSpeed << " at " << omnetpp::simTime() << "s\n";
                std::cout << "Vehicle " << mTraciId << " target: pos=(" << targetLatPos << "," << targetLonPos
                          << "), speed=" << targetLatSpeed << " at " << omnetpp::simTime() << "s\n";                

                // 目標速度に設定（0以上の値のみ）
                if (targetLatSpeed >= 0) {
                    vehicleController->setSpeed(targetLatSpeed * vanetza::units::si::meter_per_second);
                    EV_INFO << "Set speed to " << targetLatSpeed << " m/s for vehicle " << mTraciId << std::endl;
                    std::cout << "Set speed to " << targetLatSpeed << " m/s for vehicle " << mTraciId << std::endl;
                }
                
                // レーン変更が必要かどうかを確認
                double laneWidth = mLaneWidth;
                int currentLane = static_cast<int>(currentLonPos / laneWidth);
                int targetLane = static_cast<int>(targetLonPos / laneWidth);

                std::cout << "currentLane: " << currentLane << ", " << "targetLane: " << targetLane << std::endl;
                
                if (currentLane != targetLane) {
                    // レーン変更を実行
                    changeLane(vehicleController, targetLane);
                }
            }
        } else {
            EV_ERROR << "Vehicle controller not available" << std::endl;
        }
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
        mMaxSpeed, laneCenterPositions, mConvTime
    );
    pathCandidates.insert(pathCandidates.end(), speedPathCandidates.begin(), speedPathCandidates.end());
    
    // 前方車両が存在するレーンに対して、目標位置に達する候補経路を生成
    for (double laneCenterPos : laneCenterPositions) {
        double leadingVehiclePos = getLeadingVehiclePosition(laneCenterPos);
    
        std::cout << mTraciId << ": leadingVehiclePos: " << leadingVehiclePos << " m" << std::endl;

        if (leadingVehiclePos > 0) { // 前方車両が存在する場合
            double leadingVehicleSpeed = getLeadingVehicleSpeed(laneCenterPos);
            // 先行車両の5秒後の予測位置 = 現在位置 + 速度 * 時間
            double predictedLeadingVehiclePos = leadingVehiclePos + (leadingVehicleSpeed * mConvTime);
            // 目標位置 = 先行車両の5秒後の予測位置 - 自車両の長さ - 安全距離
            double targetPos = predictedLeadingVehiclePos - mVehicleLength - mSafetyDistance;

            std::cout << mTraciId << ": leadingVehicleSpeed: " << leadingVehicleSpeed << " m/s" << std::endl;
            std::cout << mTraciId << ": targetPos: " << targetPos << " m" << std::endl; 

            auto posPathCandidates = mPlanner.generateMaxPosPathCandidates(
                latPos, latSpeed, latAccel,
                lonPos, lonSpeed, lonAccel,
                targetPos, leadingVehicleSpeed, { laneCenterPos }, mConvTime
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
    
    // 軌道の長さは同じであるため最小値を使用
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
    double leadingVehicleSpeed = mMaxSpeed; // デフォルトは最大速度
    
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

// レーン変更ヘルパーメソッド
void ManeuverCoordinationService::changeLane(traci::VehicleController* controller, int targetLane)
{
    try {
        if (!controller) return;
        
        // 現在走行中の道路（エッジ）を取得
        std::string currentEdge = controller->getLiteAPI().vehicle().getRoadID(controller->getVehicleId());
        
        // 横方向位置を取得
        double lonPosition = mVehicleDataProvider->position().y.value();
        int currentLane = static_cast<int>(lonPosition / mLaneWidth);
        
        EV_INFO << "Vehicle " << mTraciId << " on edge " << currentEdge 
               << ", current lane: " << currentLane << ", target lane: " << targetLane << std::endl;
        std::cout << "Vehicle " << mTraciId << " on edge " << currentEdge 
                  << ", current lane: " << currentLane << ", target lane: " << targetLane << std::endl; 
        if (currentLane != targetLane) {
            // 方法1: changeLaneコマンドを使用（SUMOのバージョンによって動作が異なる場合があります）
            // controller->getLiteAPI().vehicle().changeLane(
            //     controller->getVehicleId(), targetLane, 5.0);  // 5.0秒で変更
            
            // 方法2: moveToCコマンドを使用して正確な位置に移動
            double newLonPos = (targetLane + 0.5) * mLaneWidth;  // レーンの中央位置
            double latPos = mVehicleDataProvider->position().x.value();
            traci::TraCIPosition position;
            position.x = latPos + 100; 
            position.y = newLonPos;
            std::cout << "latPos: " << latPos << ", newLonPos: " << newLonPos << std::endl;
            controller->getLiteAPI().vehicle().moveToXY(
                controller->getVehicleId(), currentEdge, -1, position.x, position.y, -1, 5.0);
            
            EV_INFO << "Lane change command sent for vehicle " << mTraciId << std::endl;
            std::cout << "Lane change command sent for vehicle " << mTraciId << std::endl;
        }
    } catch (const std::exception& e) {
        EV_ERROR << "Lane change failed: " << e.what() << std::endl;
    }
}

void ManeuverCoordinationService::finish()
{
    ItsG5Service::finish();
    
    if (mEnableVisualization) {
        MCMWebVisualizer::getInstance().close();
    }
}

} // namespace artery