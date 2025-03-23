// MCSモジュール
#include "artery/application/mcs/ManeuverCoordinationService.h"
#include "artery/application/mcs/ManeuverCoordinationMessage.h"
// ユーティリティモジュール
#include "artery/utility/IdentityRegistry.h"
#include "artery/utility/InitStages.h"
// ommnetppモジュール
#include <omnetpp/checkandcast.h>
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
// vanetzaモジュール
#include <vanetza/btp/ports.hpp>
// boostモジュール
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>

namespace artery
{

Define_Module(ManeuverCoordinationService)

ManeuverCoordinationService::~ManeuverCoordinationService()
{
    cancelAndDelete(mTrigger);
}

int ManeuverCoordinationService::numInitStages() const
{
    return InitStages::Total;
} 

void ManeuverCoordinationService::initialize(int stage)
{
    if (stage == InitStages::Prepare) {
        ItsG5Service::initialize();
    } else if (stage == InitStages::Self) {
        mTraciId = getFacilities().get_const<Identity>().traci;
    }
}

void ManeuverCoordinationService::trigger()
{
    using namespace vanetza;
    btp::DataRequestB req;
    req.destination_port = host_cast<PortNumber>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(0);
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    // 車両IDとバイト長を設定
    auto packet = new ManeuverCoordinationMessage();
    packet->setTraciId(mTraciId);
    packet->setByteLength(100); 
    request(req, packet);

    std::cout << omnetpp::simTime() << ": veh "<< mTraciId << " sent MCM\n";
}

void ManeuverCoordinationService::indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket* packet)
{
    auto mcm = omnetpp::check_and_cast<ManeuverCoordinationMessage*>(packet);
    std::cout << omnetpp::simTime() << ": veh "<< mTraciId << " received MCM by veh " << mcm->getTraciId() << "\n";
    delete packet;
}

void ManeuverCoordinationService::handleMessage(omnetpp::cMessage* msg)
{
    if (msg == mTrigger) {
        trigger();
    } else {
        ItsG5Service::handleMessage(msg);
    }
}

} // namespace artery
