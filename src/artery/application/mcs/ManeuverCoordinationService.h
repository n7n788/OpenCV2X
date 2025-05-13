#ifndef ARTERY_MANEUVERCOORDINATIONSERVICE_H_
#define ARTERY_MANEUVERCOORDINATIONSERVICE_H_

#include "artery/application/mcs/ManeuverCoordinationMessage.h"
#include "artery/application/mcs/FrenetPlanning.h"
#include "artery/application/mcs/MCMWebVisualizer.h"
#include "artery/application/ItsG5Service.h"
#include <map>
#include <string>
#include <vector>
#include <set>

namespace artery
{

class VehicleDataProvider;
class LocalEnvironmentModel;

class ManeuverCoordinationService : public ItsG5Service
{
public:
    virtual ~ManeuverCoordinationService();

protected:
    /*
     * 初期化ステージ数を返す
     * @return 初期化ステージ数
     */
    int numInitStages() const override;
    
    /*
     * 初期化
     * @param stage 初期化ステージ
     */
    void initialize(int stage) override;
    
    /*
     * 受信処理
     * @param indication 受信したデータ
     * @param packet 受信したパケット
     */
    void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
    
    /*
     * メッセージハンドラ
     * @param msg 受信したメッセージ
     */
    void handleMessage(omnetpp::cMessage*) override;
    
    /*
     * 送信処理（0.1秒ごとに呼び出される）
     */
    void trigger() override;

private:
    /*
     * MCMを生成
     * @return 生成されたMCM
     */
    ManeuverCoordinationMessage* generate();
    
    /*
     * 経路が衝突しているかチェック
     * @param path1 経路1
     * @param path2 経路2
     * @return 衝突している場合true
     */
    bool checkCollision(const FrenetPath& path1, const FrenetPath& path2);
    
    /*
     * senderIdの通行権が自分より優先かどうかを判定
     * @param senderId 車両ID
     * @return senderIdの方が優先度が高い場合true
     */
    bool hasPriority(const std::string& senderId, const FrenetPath& path);
    
    /*
     * 受信した経路を受け入れるかどうかの判定
     * @param senderId 送信元の車両ID
     * @param desiredPath 希望経路
     * @return 受け入れる場合true
     */
    bool acceptPath(const std::string& senderId, const FrenetPath& desiredPath);
    
    // 前方車両の位置を取得
    double getLeadingVehiclePosition(double lanePosition);
    
    // 前方車両の速度を取得
    double getLeadingVehicleSpeed(double lanePosition);
  
    /*
     * レーン変更コマンドを発行
     * @param controller 車両コントローラー
     * @param targetLane 目標レーン番号
     */
    void changeLane(traci::VehicleController* controller, int targetLane);

    /*
     * 車両が障害物かどうかを判定
     * @return 障害物の場合true
     * @note 車両IDが"obstacle"で始まる場合、障害物とみなす
    */
    bool isObstacle(const std::string&) const;
 
    void finish() override;

    std::string mTraciId; //> 車両ID
    omnetpp::cMessage* mTrigger = nullptr; //> 送信トリガー
    
    FrenetPlanning mPlanner; //> 経路計画
    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    traci::VehicleController* mVehicleController = nullptr;
    
    std::map<std::string, FrenetPath> mReceivedPlannedPaths; //> 受信した予定経路
    std::map<std::string, FrenetPath> mReceivedDesiredPaths; //> 受信した希望経路
    
    std::set<std::string> mAcceptedIds; //> 交渉受け入れリスト
    std::map<std::string, std::pair<double, double>> mVehiclePoses; //> 他車両の位置情報
    std::map<std::string, std::pair<double, double>> mVehicleSpeeds; //> 他車両の速度情報
    
    FrenetPath mPreviousPlannedPath; //> 1ステップ前の予定経路
    FrenetPath mPreviousDesiredPath; //> 1ステップ前の希望経路
    
    std::vector<FrenetPath> mPlannedPaths; //> 予定経路リスト
    std::vector<FrenetPath> mDesiredPaths; //> 希望経路リスト

    double mLastGenerateMcmTime; //> 最後にMCMを送信した時間
    double mLastLaneChangeTime; //> 最後にレーン変更した時間
    
    bool mLaneChangeInProgress = false; //> 車線変更中フラグ
    int mTargetLane = -1; //> 目標レーン
    double mLaneChangeStartTime = 0.0; //> 車線変更開始時間
    double mLastUpdateTime = 0.0; //> 最後に更新した時間

    // 設定パラメータ
    double mLaneWidth; //> レーン幅
    int mNumLanes; //> レーン数
    std::vector<double> mCenterLanes; //> 利用可能なレーン位置
    double mVehicleLength; //> 車両の長さ
    double mSafetySecond = 2.0; //> 安全距離
    double mConvTime; //> 収束時間
    double mMcmInterval = 1.0; //> MCM送信間隔
    double mLaneChangeInterval = 5.0; //> 車線変更後のインターバル
    bool mEnableVisualization = false;
  
    double mDesiredCostThreshold = 100.0; //> 希望経路のコストが予定経路のコストをこの閾値以下で下回る場合に、希望経路を生成
    const std::string mObstacle = "obstacle"; //> 障害物のID
};

} // namespace artery

#endif /* ARTERY_MANEUVERCOORDINATIONSERVICE_H_ */