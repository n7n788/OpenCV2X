#ifndef ARTERY_MANEUVERCOORDINATIONSERVICE_H_
#define ARTERY_MANEUVERCOORDINATIONSERVICE_H_

#include "artery/application/mcs/ManeuverCoordinationMessage.h"
#include "artery/application/mcs/PathGenerator.h"
#include "artery/application/mcs/PathPlanner.h"
#include "artery/application/mcs/CollisionDetector.h"
#include "artery/application/mcs/NegotiationManager.h"
#include "artery/application/mcs/VehicleControllerWrapper.h"
#include "artery/application/mcs/MCMWebVisualizer.h"
#include "artery/application/ItsG5Service.h"
#include <map>
#include <string>
#include <vector>
#include <set>
#include <memory>

namespace artery
{

class VehicleDataProvider;

/**
 * @brief 車両間協調制御サービス
 * 
 * V2X通信を用いて車両間で経路情報を共有し、協調的な運転操作を実現する。
 * 各専門クラスに責任を委譲し、自身は通信と全体制御に専念する。
 */
class ManeuverCoordinationService : public ItsG5Service
{
public:
    virtual ~ManeuverCoordinationService();

protected:
    /**
     * @brief 初期化ステージ数を返す
     * @return 初期化ステージ数
     */
    int numInitStages() const override;
    
    /**
     * @brief サービスの初期化
     * @param stage 初期化ステージ
     */
    void initialize(int stage) override;
    
    /**
     * @brief MCMメッセージの受信処理
     * @param indication 受信データ情報
     * @param packet 受信パケット
     */
    void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
    
    /**
     * @brief メッセージハンドラ
     * @param msg 受信メッセージ
     */
    void handleMessage(omnetpp::cMessage*) override;
    
    /**
     * @brief 定期送信処理
     */
    void trigger() override;

private:
    /**
     * @brief MCMメッセージの生成
     * @return 生成されたMCMメッセージ
     */
    ManeuverCoordinationMessage* generate();
    
    /**
     * @brief 終了処理
     */
    void finish() override;

    /**
     * @brief 2つの経路が同じかどうかを判定
     * @param prevPath [Path] 1つ前の経路
     * @param currentPath [Path] 現在の経路
     * @return [bool] 同じ経路ならtrue、異なるならfalse
    */
   bool samePath(const Path& prevPath, const Path& currentPath) const;

    // ========== 専門クラス ==========
    
    std::unique_ptr<PathGenerator> mPathGenerator;               ///< 基本経路生成エンジン
    std::unique_ptr<PathPlanner> mPathPlanner;                   ///< 高レベル経路計画器
    std::unique_ptr<CollisionDetector> mCollisionDetector;       ///< 衝突判定器
    std::unique_ptr<NegotiationManager> mNegotiationManager;     ///< 交渉管理器
    std::unique_ptr<VehicleControllerWrapper> mVehicleWrapper;   ///< 車両制御ラッパー

    // ========== 基本システム ==========
    
    std::string mTraciId;                                        ///< 自車両ID
    omnetpp::cMessage* mTrigger = nullptr;                       ///< 送信トリガー
    const VehicleDataProvider* mVehicleDataProvider = nullptr;   ///< 車両状態プロバイダ
    traci::VehicleController* mVehicleController = nullptr;      ///< 車両コントローラー
    
    // ========== 通信データ ==========
    
    std::map<std::string, Path> mReceivedPlannedPaths;           ///< 受信予定経路
    std::map<std::string, Path> mReceivedDesiredPaths;           ///< 受信希望経路
    std::set<std::string> mAcceptedIds;                          ///< 交渉受け入れリスト
    std::map<std::string, std::pair<double, double>> mVehiclePoses;  ///< 他車両位置情報
    std::map<std::string, std::pair<double, double>> mVehicleSpeeds; ///< 他車両速度情報
    
    // ========== 経路履歴 ==========
    
    Path mPreviousPlannedPath;                                   ///< 前回予定経路
    Path mPreviousDesiredPath;                                   ///< 前回希望経路
    std::vector<Path> mPlannedPaths;                             ///< 予定経路履歴
    std::vector<Path> mDesiredPaths;                             ///< 希望経路履歴

    // ========== タイミング制御 ==========
    
    double mLastGenerateMcmTime;                                 ///< 最後のMCM送信時刻
    double mLastLaneChangeTime;                                  ///< 最後の車線変更時刻
    double mLastUpdateTime = 0.0;                                ///< 最後の更新時刻

    // ========== 設定パラメータ ==========
    
    double mLaneWidth;                                           ///< レーン幅 [m]
    int mNumLanes;                                               ///< レーン数
    std::vector<double> mCenterLanes;                            ///< レーン中央位置（互換性保持）
    double mVehicleLength;                                       ///< 車両長 [m]
    double mSafetySecond;                                        ///< 安全時間 [s]
    double mConvTime;                                            ///< 収束時間 [s]
    double mLaneChangeInterval;                                  ///< 車線変更インターバル [s]
    bool mEnableVisualization;                                   ///< 可視化有効フラグ
    double mDesiredCostThreshold;                                ///< 希望経路コスト閾値
    const std::string mObstacle = "obstacle";                    ///< 障害物ID接頭辞

    // ========= 評価を取るための変数 ==========

    double mEntryTime;                                       ///< 道路への進入時間
    double mExitLonPos;                                      ///< 道路からの退出する縦方向の位置
    double mNegotiationStartTime = -1.0;                          ///< 交渉開始時間
    double mNegotiationTimeout;                         ///< 交渉タイムアウト時間

    // ======== シグナル =======

    omnetpp::simsignal_t velocitySignal;                  ///< 速度シグナル
    omnetpp::simsignal_t travelTimeSignal;                 ///< 移動時間シグナル
    omnetpp::simsignal_t negotiationTimeSignal;         ///< 交渉時間シグナル
    omnetpp::simsignal_t negotiationResultSignal;    ///< 交渉結果シグナル
};

} // namespace artery

#endif
