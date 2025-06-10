#include "artery/application/mcs/ManeuverCoordinationService.h"
#include "artery/application/mcs/ManeuverCoordinationMessage.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/Identity.h"
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
        
        // パラメータ読み込み
        mLaneWidth = par("laneWidth").doubleValue();
        mNumLanes = par("numLanes").intValue();
        mVehicleLength = par("vehicleLength").doubleValue();
        mSafetySecond = par("safetySecond").doubleValue();
        mConvTime = par("convergenceTime").doubleValue();
        mLaneChangeInterval = par("laneChangeInterval").doubleValue();
        mDesiredCostThreshold = par("desiredCostThreshold").doubleValue();
        
        // レーン位置設定（互換性保持）
        for (int i = 0; i < mNumLanes; i++) {
            mCenterLanes.push_back(i * mLaneWidth + mLaneWidth / 2.0);
        }
        
        // 専門クラスの初期化（依存関係の順序に注意）
        mPathGenerator.reset(new PathGenerator());
        mCollisionDetector.reset(new CollisionDetector(mVehicleLength, mLaneWidth, mSafetySecond));
        mNegotiationManager.reset(new NegotiationManager(*mCollisionDetector));
        mPathPlanner.reset(new PathPlanner(*mPathGenerator, *mCollisionDetector, mLaneWidth, mNumLanes, mVehicleLength, mSafetySecond, mConvTime));
        
        // 基本システムの初期化
        mTrigger = new omnetpp::cMessage("MCS trigger");
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        mVehicleController = getFacilities().get_mutable_ptr<traci::VehicleController>();
        
        // 車両制御ラッパーの初期化
        mVehicleWrapper.reset(new VehicleControllerWrapper(
            mVehicleController, mVehicleDataProvider, mLaneWidth, mNumLanes, mConvTime));
        
        mLastUpdateTime = omnetpp::simTime().dbl();
        
        // 可視化設定
        mEnableVisualization = par("enableVisualization").boolValue();
        if (mEnableVisualization) {
            try {
                MCMWebVisualizer::getInstance().initialize(this, par("visualizationPort").intValue());
                EV_INFO << "MCM visualization enabled" << std::endl;
            } catch (const std::exception& e) {
                EV_ERROR << "Failed to initialize MCM visualization: " << e.what() << std::endl;
            }
        }
    }
    else if (stage == InitStages::Self) {
        mTraciId = getFacilities().get_const<Identity>().traci;
        
        double initialLatPos = mVehicleDataProvider->position().y.value();
        int initialLane = static_cast<int>(initialLatPos / mLaneWidth);
        EV_INFO << "Vehicle " << mTraciId << " starting in lane " << initialLane << std::endl;
    }
}

void ManeuverCoordinationService::trigger()
{
    using namespace vanetza;
    
    // MCM生成
    auto mcm = generate();
 
    // 車両制御実行
    if (!mPlannedPaths.empty() && ! mCollisionDetector->isObstacle(mTraciId)) {
        mVehicleWrapper->executePath(mPlannedPaths.back(), mTraciId);
        mLastUpdateTime = omnetpp::simTime().dbl();
    }
    
    // BTP設定と送信
    btp::DataRequestB req;
    req.destination_port = host_cast<PortNumber>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
    
    request(req, mcm);
    EV_INFO << "Vehicle " << mTraciId << " sent MCM" << std::endl;
}

ManeuverCoordinationMessage* ManeuverCoordinationService::generate()
{
    // 1. 候補経路生成
    std::vector<Path> pathCandidates = mPathPlanner->generateCandidatePaths(
        mVehicleDataProvider,
        mVehicleController,
        mVehiclePoses,
        mVehicleSpeeds,
        mTraciId,
        mVehicleWrapper->getLastLaneChangeTime(),
        mLaneChangeInterval
    );

    // 2. 交渉処理
    mNegotiationManager->processReceivedDesiredPaths(
        mReceivedDesiredPaths, mPreviousPlannedPath, mAcceptedIds);
    
    // 3. 最適経路選択
    Path plannedPath = mPathPlanner->selectPlannedPath(
        pathCandidates, mReceivedPlannedPaths, mReceivedDesiredPaths, 
        mAcceptedIds, mTraciId, mVehiclePoses, mVehicleDataProvider);

    Path desiredPath = mPathPlanner->selectDesiredPath(
        pathCandidates, mReceivedPlannedPaths, mReceivedDesiredPaths,
        mAcceptedIds, plannedPath, mDesiredCostThreshold, mTraciId);

    // 4. フォールバック経路
    if (plannedPath.getCost() < 0) {
        double lonPos = mVehicleDataProvider->position().x.value();
        double lonSpeed = mVehicleDataProvider->speed().value();
        double lonAccel = mVehicleDataProvider->acceleration().value();
        double latPos = mVehicleDataProvider->position().y.value();
        
        plannedPath = mPathGenerator->generateSpeedPath(
            lonPos, lonSpeed, lonAccel, latPos, 0.0, 0.0, lonSpeed, mConvTime);
    }

    // 5. MCM作成
    auto mcm = new ManeuverCoordinationMessage();
    mcm->setTraciId(mTraciId);
    mcm->setPlannedPath(plannedPath);
    mcm->setDesiredPath(desiredPath);
    mcm->setLonPos(mVehicleDataProvider->position().x.value());
    mcm->setLonSpeed(mVehicleDataProvider->speed().value());
    mcm->setLonAccel(mVehicleDataProvider->acceleration().value());
    mcm->setLatPos(mVehicleDataProvider->position().y.value());
    mcm->setLatSpeed(0.0);
    mcm->setLatAccel(0.0);
    mcm->setByteLength(200);
    
    // 6. 状態更新
    mPlannedPaths.push_back(plannedPath);
    mDesiredPaths.push_back(desiredPath);
    mPreviousPlannedPath = plannedPath;
    mPreviousDesiredPath = desiredPath;
    mLastGenerateMcmTime = omnetpp::simTime().dbl();
    
    // 7. 可視化更新
    if (mEnableVisualization) {
        MCMWebVisualizer::getInstance().setEgoPaths(mTraciId, plannedPath, desiredPath);
        MCMWebVisualizer::getInstance().setPathCandidates(mTraciId, pathCandidates);
    }
    
    return mcm;
}

void ManeuverCoordinationService::indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket* packet)
{
    auto mcm = omnetpp::check_and_cast<ManeuverCoordinationMessage*>(packet);
    const std::string& senderId = mcm->getTraciId();
    
    EV_INFO << "Vehicle " << mTraciId << " received MCM from " << senderId << std::endl;
    
    // 受信データ保存
    mReceivedPlannedPaths[senderId] = mcm->getPlannedPath();
    mReceivedDesiredPaths[senderId] = mcm->getDesiredPath();
    mVehiclePoses[senderId] = std::make_pair(mcm->getLonPos(), mcm->getLatPos());
    mVehicleSpeeds[senderId] = std::make_pair(mcm->getLonSpeed(), mcm->getLatSpeed());
    
    // 可視化更新
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

void ManeuverCoordinationService::finish()
{
    ItsG5Service::finish();
    
    if (mEnableVisualization) {
        MCMWebVisualizer::getInstance().close();
    }
}

} // namespace artery
