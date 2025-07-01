#ifndef ARTERY_NEGOTIATION_MANAGER_H
#define ARTERY_NEGOTIATION_MANAGER_H

#include "Path.h"
#include "CollisionDetector.h"
#include <set>
#include <map>
#include <vector>
#include <string>

namespace artery {

/**
 * @brief 車両間の経路交渉を管理するクラス
 * 
 * 受信した希望経路に対する受け入れ判定と、
 * 交渉状態の管理を行う。
 */
class NegotiationManager {
public:
    /**
     * @brief コンストラクタ
     * @param detector 衝突判定器への参照
     */
    NegotiationManager(const CollisionDetector& detector);
    
    /**
     * @brief 受信した希望経路を処理し交渉を行う
     * @param receivedDesiredPaths 受信希望経路
     * @param myPreviousPlannedPath 自車両の前回予定経路
     * @param candidates 候補経路リスト
     * @param traciId 自車両のID
     * @param acceptedIds 交渉受け入れリスト（更新される）
     */
    void processReceivedDesiredPaths(
        const std::map<std::string, Path>& receivedDesiredPaths,
        const Path& myPreviousPlannedPath,
        const std::vector<Path>& candidates,
        const std::string& traciId,
        std::set<std::string>& acceptedIds
    );
    
private:
    const CollisionDetector& mDetector; ///< 衝突判定器への参照
    
    /**
     * @brief 経路を受け入れるかの判定
     * @param senderId 送信者ID
     * @param desiredPath 希望経路
     * @return 受け入れる場合true
     */
    bool acceptPath(const std::string& senderId, const Path& desiredPath);

    /**
     * @brief 希望経路を受け入れ可能な候補経路があるかを判定
     * @param desiredPath 受信希望経路
     * @param candidates 候補経路
     * @return 受け入れ可能な候補がある場合true
    */
    bool canAcceptAnyPath(const Path& desiredPath,
                          const std::vector<Path>& candidates) const;

};

} // namespace artery

#endif
