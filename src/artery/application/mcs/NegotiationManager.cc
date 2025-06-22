#include "NegotiationManager.h"
#include <vector>
#include <iostream>

namespace artery {

NegotiationManager::NegotiationManager(const CollisionDetector& detector)
    : mDetector(detector) {
}

void NegotiationManager::processReceivedDesiredPaths(
    const std::map<std::string, Path>& receivedDesiredPaths,
    const Path& myPreviousPlannedPath,
    const std::vector<Path>& candidates,
    std::set<std::string>& acceptedIds) {
    
    for (const auto& entry : receivedDesiredPaths) {
        const std::string& senderId = entry.first;
        const Path& desiredPath = entry.second;
        
        // 自身の予定経路と希望経路が衝突しており、
        // 衝突しない候補経路が存在し、
        // 交渉を受け入れる判断をする場合、
        // 交渉を受け入れる 
        if (!myPreviousPlannedPath.getLonTrajectory().getPoses().empty() && 
            mDetector.checkCollision(myPreviousPlannedPath, desiredPath) &&
            canAcceptAnyPath(desiredPath, candidates) &&
            acceptPath(senderId, desiredPath)) {
            
            acceptedIds.insert(senderId);
        }
    }
}

bool NegotiationManager::acceptPath(const std::string& senderId, const Path& desiredPath) {
    return true;
}

bool NegotiationManager::canAcceptAnyPath(const Path& desiredPath,
                      const std::vector<Path>& candidates) const {
    for (const auto& candidate : candidates) {
        if (!mDetector.checkCollision(desiredPath, candidate)) {
            return true; // 受け入れ可能な候補が見つかった
        }
    }
    return false; // 受け入れ可能な候補がない
}

} // namespace artery
