// ManeuverCoordinationMessage.cc
#include "artery/application/mcs/ManeuverCoordinationMessage.h"
#include <omnetpp.h>

namespace artery
{

Register_Class(ManeuverCoordinationMessage)

ManeuverCoordinationMessage::ManeuverCoordinationMessage() :
    omnetpp::cPacket("Maneuver Coordination Message"),
    mLatPos(0.0),
    mLatSpeed(0.0),
    mLatAccel(0.0),
    mLonPos(0.0),
    mLonSpeed(0.0),
    mLonAccel(0.0)
{
}

omnetpp::cPacket* ManeuverCoordinationMessage::dup() const
{
    return new ManeuverCoordinationMessage(*this);
}

} // namespace artery