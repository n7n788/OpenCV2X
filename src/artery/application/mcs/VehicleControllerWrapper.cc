#include "VehicleControllerWrapper.h"
#include <boost/units/systems/si.hpp>
#include <iostream>
#include <omnetpp.h>

namespace artery {

VehicleControllerWrapper::VehicleControllerWrapper(
    traci::VehicleController* controller,
    const VehicleDataProvider* dataProvider,
    double laneWidth, int numLanes, double convTime)
    : mController(controller), mDataProvider(dataProvider),
      mLaneWidth(laneWidth), mNumLanes(numLanes), mConvTime(convTime) {
}

void VehicleControllerWrapper::executePath(const Path& plannedPath, const std::string& traciId) {
    if (plannedPath.getLonTrajectory().getPoses().empty()) {
        return;
    }
    
    handleLaneChange(plannedPath, traciId);
    handleSpeedControl(plannedPath);
}

void VehicleControllerWrapper::handleLaneChange(const Path& plannedPath, const std::string& traciId) {
    double currentLatPos = mDataProvider->position().y.value();
    int currentLane = static_cast<int>((mLaneWidth * mNumLanes - currentLatPos) / mLaneWidth);
    
    const auto& latPoses = plannedPath.getLatTrajectory().getPoses();
    if (latPoses.empty()) return;
    
    double targetLatPos = latPoses.back();
    int targetLane = static_cast<int>((mLaneWidth * mNumLanes - targetLatPos) / mLaneWidth);
    
    if (!mLaneChangeInProgress && currentLane != targetLane) {
        auto& api = mController->getLiteAPI();
        api.vehicle().changeLane(traciId, targetLane, mConvTime);
        
        // std::cout << "Vehicle " << traciId << " changing lane from " 
        //           << currentLane << " to " << targetLane 
        //           << " over " << mConvTime << " seconds\n";
        
        mLaneChangeInProgress = true;
        mTargetLane = targetLane;
        mLaneChangeStartTime = omnetpp::simTime().dbl();
    }
    
    if (mLaneChangeInProgress) {
        double elapsedTime = omnetpp::simTime().dbl() - mLaneChangeStartTime;
        
        if (elapsedTime >= mConvTime || currentLane == mTargetLane) {
            mLaneChangeInProgress = false;
            mLastLaneChangeTime = omnetpp::simTime().dbl();
            // std::cout << "Vehicle " << traciId << " completed lane change to " 
            //           << mTargetLane << " time: " << mLastLaneChangeTime << "\n";
        }
    }
}

void VehicleControllerWrapper::handleSpeedControl(const Path& plannedPath) {
    const auto& lonSpeeds = plannedPath.getLonTrajectory().getSpeeds();
    if (lonSpeeds.empty()) return;
    
    int index = 1;
    if (index < static_cast<int>(lonSpeeds.size())) {
        double targetSpeed = lonSpeeds[index];
        mController->setSpeed(targetSpeed * boost::units::si::meter_per_second);
    }
}

} // namespace artery
