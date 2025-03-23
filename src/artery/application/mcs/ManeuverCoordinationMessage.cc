#include "artery/application/mcs/ManeuverCoordinationMessage.h"
#include <omnetpp.h>


namespace artery
{

Register_Class(ManeuverCoordinationMessage)

ManeuverCoordinationMessage::ManeuverCoordinationMessage() :
    omnetpp::cPacket("Maneuver Coordination Message")
{
}

omnetpp::cPacket* ManeuverCoordinationMessage::dup() const
{
    return new ManeuverCoordinationMessage(*this);
}

}

