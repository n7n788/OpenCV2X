/*
* miyata 2024/07/01
*/

#ifndef ARTERY_MANEUVERCOORDINATIONMESSAGE_H
#define ARTERY_MANEUVERCOORDINATIONMESSAGE_H

#include <omnetpp/cpacket.h>
#include <string>
#include <vector>

namespace artery
{
// class FrenetPath;

class ManeuverCoordinationMessage : public omnetpp::cPacket
{
public:

    /*
        * コンストラクタ
    */
    ManeuverCoordinationMessage();
    
    /*
        * コピーコンストラクタ
    */
    ManeuverCoordinationMessage(const ManeuverCoordinationMessage&) = default;
    
    /*
        * 代入演算子
    */
    ManeuverCoordinationMessage& operator=(const ManeuverCoordinationMessage&) = default;

    omnetpp::cPacket* dup() const override;

    // getter
    /*
        * 車両IDを取得
        * @return 車両ID
    */
    const std::string& getTraciId() const { return mTraciId; }

    // setter
    /*
        * 車両IDを設定
        * @param traciId 車両ID
    */
    void setTraciId(std::string traciId) { mTraciId = traciId; }

private:
    std::string mTraciId; //> 車両ID
};

}
#endif /* ARTERY_MANEUVERCOORDINATIONMESSAGE_H */ 
