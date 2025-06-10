#include "NegotiationManager.h"
#include <iostream>

namespace artery {

NegotiationManager::NegotiationManager(const CollisionDetector& detector)
    : mDetector(detector) {
}

void NegotiationManager::processReceivedDesiredPaths(
    const std::map<std::string, Path>& receivedDesiredPaths,
    const Path& myPreviousPlannedPath,
    std::set<std::string>& acceptedIds) {
    
    for (const auto& entry : receivedDesiredPaths) {
        const std::string& senderId = entry.first;
        const Path& desiredPath = entry.second;
        
        if (!myPreviousPlannedPath.getLonTrajectory().getPoses().empty() && 
            mDetector.checkCollision(myPreviousPlannedPath, desiredPath)) {
            
            if (acceptPath(senderId, desiredPath)) {
                acceptedIds.insert(senderId);
            }
        }
    }
}

bool NegotiationManager::acceptPath(const std::string& senderId, const Path& desiredPath) {
    return true;
}

} // namespace artery
