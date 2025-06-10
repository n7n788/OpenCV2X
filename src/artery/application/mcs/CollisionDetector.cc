#include "CollisionDetector.h"
#include <cmath>
#include <algorithm>

namespace artery {

CollisionDetector::CollisionDetector(double vehicleLength, double laneWidth, double safetySecond)
    : mVehicleLength(vehicleLength), mLaneWidth(laneWidth), mSafetySecond(safetySecond) {
}

bool CollisionDetector::checkCollision(const Path& path1, const Path& path2) const {
    const double LON_COLLISION_THRESHOLD = mVehicleLength / 2;
    const double LAT_COLLISION_THRESHOLD = mLaneWidth / 2;
    
    const auto& lon1 = path1.getLonTrajectory().getPoses();
    const auto& lon1Speeds = path1.getLonTrajectory().getSpeeds();
    const auto& lat1 = path1.getLatTrajectory().getPoses();
    const auto& lon2 = path2.getLonTrajectory().getPoses();
    const auto& lat2 = path2.getLatTrajectory().getPoses();
    
    if (lon1.empty() || lat1.empty() || lon2.empty() || lat2.empty()) {
        return false;
    }
    
    size_t size = std::min({lon1.size(), lat1.size(), lon2.size(), lat2.size()});
    
    for (size_t i = 0; i < size; i++) {
        double dlon = lon1[i] - lon2[i];
        double dlat = lat1[i] - lat2[i];
        
        if (abs(dlon) < LON_COLLISION_THRESHOLD + lon1Speeds[i] * mSafetySecond && 
            abs(dlat) < LAT_COLLISION_THRESHOLD) {
            return true;
        }
    }
    
    return false;
}

bool CollisionDetector::hasPriority(const std::string& senderId, const Path& myPath,
                                   const std::map<std::string, std::pair<double, double>>& vehiclePoses,
                                   const VehicleDataProvider* vehicleDataProvider) const {
    if (isObstacle(senderId)) return true;

    auto it = vehiclePoses.find(senderId);
    if (it == vehiclePoses.end()) return false;

    const auto& pos = it->second;
    double lonPos = pos.first;
    double latPos = pos.second;
    double myLonPos = vehicleDataProvider->position().x.value();
    double myLatPos = vehicleDataProvider->position().y.value();
    double myLastLatPos = myPath.getLatTrajectory().getPoses().back(); 

    if (std::abs(myLastLatPos - myLatPos) > mLaneWidth / 2) {
        if (std::abs(latPos - myLastLatPos) < mLaneWidth / 2) return true;
        if (myLatPos < latPos && latPos < myLastLatPos) return true;
        if (myLatPos > latPos && latPos > myLastLatPos) return true;
    }
    
    if (std::abs(myLastLatPos - myLatPos) < mLaneWidth / 2 && 
        std::abs(latPos - myLastLatPos) < mLaneWidth / 2) {
        if (lonPos > myLonPos) {
            return true;
        }
    }

    return false;
}

bool CollisionDetector::isObstacle(const std::string& traciId) const {
    return traciId.compare(0, 8, "obstacle") == 0;
}

} // namespace artery
