#include "PathPlanner.h"
#include <algorithm>
#include <limits>
#include <omnetpp.h>

namespace artery {

PathPlanner::PathPlanner(PathGenerator& generator, const CollisionDetector& detector,
                        double laneWidth, int numLanes, 
                        double vehicleLength, double safetySecond, double convTime)
    : mGenerator(generator), mCollisionDetector(detector), mLaneWidth(laneWidth), mNumLanes(numLanes),
      mVehicleLength(vehicleLength), mSafetySecond(safetySecond), mConvTime(convTime) {
    
    for (int i = 0; i < mNumLanes; i++) {
        mCenterLanes.push_back(i * mLaneWidth + mLaneWidth / 2.0);
    }
}

std::vector<Path> PathPlanner::generateCandidatePaths(
    const VehicleDataProvider* vehicleDataProvider,
    traci::VehicleController* vehicleController,
    const std::map<std::string, std::pair<double, double>>& vehiclePoses,
    const std::map<std::string, std::pair<double, double>>& vehicleSpeeds,
    const std::string& myTraciId,
    double lastLaneChangeTime,
    double laneChangeInterval) {
    
    double lonPos = vehicleDataProvider->position().x.value();
    double lonSpeed = vehicleDataProvider->speed().value();
    double lonAccel = vehicleDataProvider->acceleration().value();
    double latPos = vehicleDataProvider->position().y.value();
    double latSpeed = 0.0;
    double latAccel = 0.0;
    double maxSpeed = vehicleController->getMaxSpeed().value();
    
    std::vector<Path> pathCandidates;
    
    // 車線変更可能なレーンの中心位置を取得
    std::vector<double> laneCenterPositions = getAvailableLanes(
        vehicleDataProvider, lastLaneChangeTime, laneChangeInterval);

    // 自由走行の経路候補を生成
    auto speedPathCandidates = mGenerator.generateMaxSpeedPathCandidates(
        lonPos, lonSpeed, lonAccel,
        latPos, latSpeed, latAccel,
        maxSpeed, laneCenterPositions, mConvTime
    );
    pathCandidates.insert(pathCandidates.end(), speedPathCandidates.begin(), speedPathCandidates.end());
    
    std::vector<double> leadingLonPoses;
    std::vector<double> leadingLonSpeeds;
    std::vector<double> leadingLatPoses;
    
    for (double laneCenterPos : laneCenterPositions) {
        double leadingVehiclePos = getLeadingVehiclePosition(laneCenterPos, vehiclePoses, vehicleDataProvider, myTraciId);
        if (leadingVehiclePos > 0) {
            double leadingVehicleSpeed = getLeadingVehicleSpeed(laneCenterPos, vehiclePoses, vehicleSpeeds, vehicleDataProvider, myTraciId);
            double predictedLeadingVehiclePos = leadingVehiclePos + (leadingVehicleSpeed * mConvTime);
            double maxTargetPos = predictedLeadingVehiclePos - mVehicleLength - lonSpeed * mSafetySecond;

            leadingLonPoses.push_back(leadingVehiclePos);
            leadingLonSpeeds.push_back(leadingVehicleSpeed);
            leadingLatPoses.push_back(laneCenterPos);
        }
    }

    // 前方車両に追従走行する経路候補を生成
    auto posPathCandidates = mGenerator.generateMaxPosPathCandidates(
        lonPos, lonSpeed, lonAccel,
        latPos, latSpeed, latAccel,
        leadingLonPoses, leadingLonSpeeds, maxSpeed, leadingLatPoses, mConvTime
    );
    pathCandidates.insert(pathCandidates.end(), posPathCandidates.begin(), posPathCandidates.end());

    // 急停止する経路候補を生成
    // Path stopPath = mGenerator.generateSpeedPath(
    //     lonPos, lonSpeed, lonAccel,
    //     latPos, latSpeed, latAccel, 0.0, maxSpeed, 2.0);
    // pathCandidates.insert(pathCandidates.end(), stopPath);

    return pathCandidates;
}

Path PathPlanner::selectPlannedPath(
    const std::vector<Path>& candidates,
    const std::map<std::string, Path>& receivedPlannedPaths,
    const std::map<std::string, Path>& receivedDesiredPaths,
    const std::set<std::string>& acceptedIds,
    const std::string& myTraciId,
    const std::map<std::string, std::pair<double, double>>& vehiclePoses,
    const VehicleDataProvider* vehicleDataProvider) {
    
    double minPlannedCost = std::numeric_limits<double>::max();
    Path plannedPath;
    
    for (const auto& path : candidates) {
        bool isValid = true;
        
        // 自分より通行権が高い車両の予定経路と衝突している候補経路を除外
        for (const auto& entry : receivedPlannedPaths) {
            const std::string& otherId = entry.first;
            const Path& otherPath = entry.second;
            
            if (mCollisionDetector.hasPriority(otherId, path, vehiclePoses, vehicleDataProvider) && 
                mCollisionDetector.checkCollision(path, otherPath)) {
                isValid = false;
                break;
            }
        }
        
        if (!isValid) continue;
        
        // 交渉受け入れリストに含まれた車両の予定経路と衝突している候補経路を除外
        for (const auto& id : acceptedIds) {
            const auto& plannedIt = receivedPlannedPaths.find(id);
            if (plannedIt != receivedPlannedPaths.end() && 
                mCollisionDetector.checkCollision(path, plannedIt->second)) {
                isValid = false;
                break;
            }
        }
        
        if (!isValid) continue;
        
        // 交渉受け入れリストに含まれた車両の希望経路と衝突している候補経路を除外
        for (const auto& id : acceptedIds) {
            const auto& desiredIt = receivedDesiredPaths.find(id);
            if (desiredIt != receivedDesiredPaths.end() && 
                mCollisionDetector.checkCollision(path, desiredIt->second)) {
                isValid = false;
                break;
            }
        }

        if (isValid && path.getCost() < minPlannedCost) {
            minPlannedCost = path.getCost();
            plannedPath = path;
        }
    }
    
    return plannedPath;
}

Path PathPlanner::selectDesiredPath(
    const std::vector<Path>& candidates,
    const std::map<std::string, Path>& receivedPlannedPaths,
    const std::map<std::string, Path>& receivedDesiredPaths,
    const std::set<std::string>& acceptedIds,
    const Path& plannedPath,
    double costThreshold,
    const std::string& myTraciId) {
    
    double minDesiredCost = std::numeric_limits<double>::max();
    Path desiredPath;
    
    for (const auto& path : candidates) {
        bool isValid = true;
        
        // 交渉受け入れリストに含まれた車両の希望経路と衝突している候補経路を除外
        for (const auto& id : acceptedIds) {
            const auto& it = receivedDesiredPaths.find(id);
            if (it != receivedDesiredPaths.end() && mCollisionDetector.checkCollision(path, it->second)) {
                isValid = false;
                break;
            }
        }
        
        // 交渉受け入れリストに含まれた車両の予定経路と衝突している候補経路を除外
        for (const auto& id : acceptedIds) {
            const auto& plannedIt = receivedPlannedPaths.find(id);
            if (plannedIt != receivedPlannedPaths.end() && 
                mCollisionDetector.checkCollision(path, plannedIt->second)) {
                isValid = false;
                break;
            }
        }
        

        // 障害物と衝突している候補経路を除外
        for (const auto& entry : receivedPlannedPaths) {
            const std::string& otherId = entry.first;
            const Path& otherPath = entry.second;
            
            if (mCollisionDetector.isObstacle(otherId) && mCollisionDetector.checkCollision(path, otherPath)) {
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
    if (plannedPath.getCost() - desiredPath.getCost() < costThreshold) {
        desiredPath = Path();
    }
    
    return desiredPath;
}

std::vector<double> PathPlanner::getAvailableLanes(
    const VehicleDataProvider* vehicleDataProvider,
    double lastLaneChangeTime,
    double laneChangeInterval) {
    
    std::vector<double> laneCenterPositions;
    double latPos = vehicleDataProvider->position().y.value();
    
    if (lastLaneChangeTime < laneChangeInterval || 
        omnetpp::simTime().dbl() - lastLaneChangeTime > laneChangeInterval) {
        for (double l : mCenterLanes) {
            if (abs(l - latPos) < mLaneWidth / 2.0 || 
                abs(l - latPos - mLaneWidth) < mLaneWidth / 2.0 || 
                abs(l - latPos + mLaneWidth) < mLaneWidth / 2.0) {
                laneCenterPositions.push_back(l);
            }
        }
    } else {
        laneCenterPositions.push_back(latPos); 
    }
    
    return laneCenterPositions;
}

double PathPlanner::getLeadingVehiclePosition(double lanePosition,
    const std::map<std::string, std::pair<double, double>>& vehiclePoses,
    const VehicleDataProvider* vehicleDataProvider,
    const std::string& myTraciId) {
    
    double minDistance = std::numeric_limits<double>::max();
    double leadingVehiclePos = -1;
    
    const double LANE_THRESHOLD = mLaneWidth / 2;
    
    for (const auto& entry : vehiclePoses) {
        const std::string& vehicleId = entry.first;
        const auto& pos = entry.second;
        
        if (vehicleId == myTraciId) continue;
        
        if (std::abs(pos.second - lanePosition) <= LANE_THRESHOLD) {
            double mySpeed = vehicleDataProvider->speed().value();
            double safetyMargin = mVehicleLength / 2 + mSafetySecond * mySpeed;
            double distance = pos.first - vehicleDataProvider->position().x.value();
            if (distance > safetyMargin && distance < minDistance) {
                minDistance = distance;
                leadingVehiclePos = pos.first;
            }
        }
    }
    
    return leadingVehiclePos;
}

double PathPlanner::getLeadingVehicleSpeed(double lanePosition,
    const std::map<std::string, std::pair<double, double>>& vehiclePoses,
    const std::map<std::string, std::pair<double, double>>& vehicleSpeeds,
    const VehicleDataProvider* vehicleDataProvider,
    const std::string& myTraciId) {
    
    double minDistance = std::numeric_limits<double>::max();
    double leadingVehicleSpeed = DBL_MAX;
    
    const double LANE_THRESHOLD = mLaneWidth / 2;
    
    for (const auto& entry : vehiclePoses) {
        const std::string& vehicleId = entry.first;
        const auto& pos = entry.second;
        
        if (vehicleId == myTraciId) continue;
        
        if (std::abs(pos.second - lanePosition) <= LANE_THRESHOLD) {
            double distance = pos.first - vehicleDataProvider->position().x.value();
            if (distance > 0 && distance < minDistance) {
                minDistance = distance;
                const auto& speedIt = vehicleSpeeds.find(vehicleId);
                if (speedIt != vehicleSpeeds.end()) {
                    leadingVehicleSpeed = speedIt->second.first;
                }
            }
        }
    }
    
    return leadingVehicleSpeed;
}

} // namespace artery
