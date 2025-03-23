#ifndef ARTERY_COLLECTIVEPERCEPTIONMOCKMESSAGE_H_YUFJ5PSV
#define ARTERY_COLLECTIVEPERCEPTIONMOCKMESSAGE_H_YUFJ5PSV

#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/sensor/FieldOfView.h"
#include "artery/envmod/sensor/SensorPosition.h"
#include <omnetpp/cpacket.h>
#include <memory>
#include <vector>
#include <string>

namespace artery
{

class CollectivePerceptionMockMessage : public omnetpp::cPacket
{
public:

    struct FovContainer
    {
        int sensorId = 0;
        SensorPosition position;
        FieldOfView fov;
    };

    struct ObjectContainer
    {
        omnetpp::SimTime timeOfMeasurement = omnetpp::SimTime::ZERO;
        std::string objectId;
        int sensorId = 0;
        std::weak_ptr<EnvironmentModelObject> object;
        double posX;
        double posY;
        double speed;
        double heading;
        double risk;
    };

    // 1秒ごとに送信. 各車両がセンシングしている車両idを持つ
    struct SensingIdContainer
    {
        std::weak_ptr<EnvironmentModelObject> object;
        std::string objectId;
    };

    CollectivePerceptionMockMessage();
    CollectivePerceptionMockMessage(const CollectivePerceptionMockMessage&) = default;
    CollectivePerceptionMockMessage& operator=(const CollectivePerceptionMockMessage&) = default;

    const std::vector<FovContainer>& getFovContainers() const { return *mFovContainers; }
    void setFovContainers(std::vector<FovContainer>&& fovs);
    void setFovContainers(const std::vector<FovContainer>& fovs);

    const std::vector<ObjectContainer>& getObjectContainers() const { return *mObjectContainers; }
    void setObjectContainers(std::vector<ObjectContainer>&& objs);
    void setObjectContainers(const std::vector<ObjectContainer>& objs);

    const std::vector<SensingIdContainer>& getSensingIdContainers() const { return *mSensingIdContainers; }
    void setSensingIdContainers(std::vector<SensingIdContainer>&& sics);
    void setSensingIdContainers(const std::vector<SensingIdContainer>& sics);

    void setSourceStation(int id) { mSourceStation = id; }
    int getSourceStation() const { return mSourceStation; }

    // 追加　
    void setGenerationDeltaTime(uint16_t time) { mGenerationDeltaTime = time; }
    uint16_t getGenerationDeltaTime() const { return mGenerationDeltaTime; }
     
    omnetpp::cPacket* dup() const override;

    // 追加 sumoのIDを返すメソッド
    std::string getTraciId() { return mTraciId; } 
    void setTraciId(std::string traciId) { mTraciId = traciId; }

    bool getExistFov() {return mExistFov;}

    void setPosX(double posX) {mPosX = posX;}
    double getPosX() {return mPosX;}
    void setPosY(double posY) {mPosY = posY;}
    double getPosY() {return mPosY;}
    void setSpeed(double Speed) {mSpeed = Speed;}
    double getSpeed() {return mSpeed;}
    void setHeading(double Heading) {mHeading = Heading;}
    double getHeading() {return mHeading;}
private:
    int mSourceStation = 0;
    std::string mTraciId; // 追加 sumoのID
    uint16_t mGenerationDeltaTime; //追加
    double mPosX;
    double mPosY;
    double mSpeed;
    double mHeading;
    std::shared_ptr<const std::vector<FovContainer>> mFovContainers;
    std::shared_ptr<const std::vector<ObjectContainer>> mObjectContainers;
    std::shared_ptr<const std::vector<SensingIdContainer>> mSensingIdContainers;
    bool mExistFov = false;
};

} // namespace artery

#endif /* ARTERY_COLLECTIVEPERCEPTIONMOCKMESSAGE_H_YUFJ5PSV */

