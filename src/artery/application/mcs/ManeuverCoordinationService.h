/*
* miyata 2024/07/01
*/

#ifndef ARTERY_MANEUVERCOORDINATIONSERVICE_H_
#define ARTERY_MANEUVERCOORDINATIONSERVICE_H_

#include "artery/application/mcs/ManeuverCoordinationMessage.h"
#include "artery/application/ItsG5Service.h"
// #include "artery/application/FrenetPlanning.h"
#include <map>
#include <string>

namespace artery
{
// class ItsG5Service;
// class PathPlannning;
// class FrenetPath;

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

        void handleMessage(omnetpp::cMessage*) override;

        /*
            * 送信処理 0.1sごとに呼び出される
        */
        void trigger() override; // 送信処理

    private:
        std::string mTraciId; //>  車両ID
        omnetpp::cMessage* mTrigger = nullptr; //> 送信トリガー
};

} // namespace artery

#endif /* ARTERY_MANEUVERCOORDINATIONSERVICE_H_ */
