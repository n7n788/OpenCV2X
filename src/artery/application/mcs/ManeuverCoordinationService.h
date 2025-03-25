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
     * 通行権の優先度を比較
     * @param id1 車両ID1
     * @param id2 車両ID2
     * @return id1の方が優先度が高い場合true
     */
    bool hasPriority(const std::string& id1, const std::string& id2);
    
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
 
    void finish() override;

    std::string mTraciId; //> 車両ID
    omnetpp::cMessage* mTrigger = nullptr; //> 送信トリガー
    
    FrenetPlanning mPlanner; //> 経路計画
    
    std::map<std::string, FrenetPath> mReceivedPlannedPaths; //> 受信した予定経路
    std::map<std::string, FrenetPath> mReceivedDesiredPaths; //> 受信した希望経路
    
    std::set<std::string> mAcceptedIds; //> 交渉受け入れリスト
    std::map<std::string, std::pair<double, double>> mVehiclePoses; //> 他車両の位置情報
    std::map<std::string, std::pair<double, double>> mVehicleSpeeds; //> 他車両の速度情報
    
    FrenetPath mPreviousPlannedPath; //> 1ステップ前の予定経路
    FrenetPath mPreviousDesiredPath; //> 1ステップ前の希望経路
    
    std::vector<FrenetPath> mPlannedPaths; //> 予定経路リスト
    std::vector<FrenetPath> mDesiredPaths; //> 希望経路リスト
    
    // 設定パラメータ
    double mMaxSpeed; //> 最大速度
    double mLaneWidth; //> レーン幅
    std::vector<double> mAvailableLanes; //> 利用可能なレーン位置
    double mVehicleLength; //> 車両の長さ
    double mSafetyDistance; //> 安全距離
    double mConvTime; //> 収束時間
    
    // 車両データプロバイダ
    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    bool mEnableVisualization = false;
};

} // namespace artery

#endif /* ARTERY_MANEUVERCOORDINATIONSERVICE_H_ */