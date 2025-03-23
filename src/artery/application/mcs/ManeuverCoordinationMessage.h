#ifndef ARTERY_MANEUVERCOORDINATIONMESSAGE_H
#define ARTERY_MANEUVERCOORDINATIONMESSAGE_H

#include <omnetpp/cpacket.h>
#include <string>
#include <vector>
#include "artery/application/mcs/FrenetPath.h"

namespace artery
{

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
    
    /*
     * パケットの複製
     */
    omnetpp::cPacket* dup() const override;
    
    // ゲッター
    /*
     * 車両IDを取得
     * @return 車両ID
     */
    const std::string& getTraciId() const { return mTraciId; }
    
    /*
     * 予定経路を取得
     * @return 予定経路
     */
    const FrenetPath& getPlannedPath() const { return mPlannedPath; }
    
    /*
     * 希望経路を取得
     * @return 希望経路
     */
    const FrenetPath& getDesiredPath() const { return mDesiredPath; }
    
    /*
     * 現在の縦方向位置を取得
     * @return 縦方向位置
     */
    double getLatPos() const { return mLatPos; }
    
    /*
     * 現在の縦方向速度を取得
     * @return 縦方向速度
     */
    double getLatSpeed() const { return mLatSpeed; }
    
    /*
     * 現在の縦方向加速度を取得
     * @return 縦方向加速度
     */
    double getLatAccel() const { return mLatAccel; }
    
    /*
     * 現在の横方向位置を取得
     * @return 横方向位置
     */
    double getLonPos() const { return mLonPos; }
    
    /*
     * 現在の横方向速度を取得
     * @return 横方向速度
     */
    double getLonSpeed() const { return mLonSpeed; }
    
    /*
     * 現在の横方向加速度を取得
     * @return 横方向加速度
     */
    double getLonAccel() const { return mLonAccel; }
    
    // セッター
    /*
     * 車両IDを設定
     * @param traciId 車両ID
     */
    void setTraciId(std::string traciId) { mTraciId = traciId; }
    
    /*
     * 予定経路を設定
     * @param path 予定経路
     */
    void setPlannedPath(const FrenetPath& path) { mPlannedPath = path; }
    
    /*
     * 希望経路を設定
     * @param path 希望経路
     */
    void setDesiredPath(const FrenetPath& path) { mDesiredPath = path; }
    
    /*
     * 現在の縦方向位置を設定
     * @param pos 縦方向位置
     */
    void setLatPos(double pos) { mLatPos = pos; }
    
    /*
     * 現在の縦方向速度を設定
     * @param speed 縦方向速度
     */
    void setLatSpeed(double speed) { mLatSpeed = speed; }
    
    /*
     * 現在の縦方向加速度を設定
     * @param accel 縦方向加速度
     */
    void setLatAccel(double accel) { mLatAccel = accel; }
    
    /*
     * 現在の横方向位置を設定
     * @param pos 横方向位置
     */
    void setLonPos(double pos) { mLonPos = pos; }
    
    /*
     * 現在の横方向速度を設定
     * @param speed 横方向速度
     */
    void setLonSpeed(double speed) { mLonSpeed = speed; }
    
    /*
     * 現在の横方向加速度を設定
     * @param accel 横方向加速度
     */
    void setLonAccel(double accel) { mLonAccel = accel; }

private:
    std::string mTraciId; //> 車両ID
    FrenetPath mPlannedPath; //> 予定経路
    FrenetPath mDesiredPath; //> 希望経路
    double mLatPos; //> 現在の縦方向位置
    double mLatSpeed; //> 現在の縦方向速度
    double mLatAccel; //> 現在の縦方向加速度
    double mLonPos; //> 現在の横方向位置
    double mLonSpeed; //> 現在の横方向速度
    double mLonAccel; //> 現在の横方向加速度
};

} // namespace artery

#endif /* ARTERY_MANEUVERCOORDINATIONMESSAGE_H */