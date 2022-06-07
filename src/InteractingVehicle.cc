//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

// setTimer()


#include "InteractingVehicle.h"
Define_Module(InteractingVehicle);

void InteractingVehicle::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        // Initializing members and pointers of your application goes here
        modulePtr = getParentModule();
        // modulePtr = veins::TraCIMobilityAccess().get(getParentModule())->getParentModule(); Pointing at node[i]
        vehicleId = modulePtr->getId();
        vehicleIndex = modulePtr->getIndex();
        // vehicleId, refer to the log left side on simulation
        // vehicleIndex is the index of the node in the vector.
        mobilityId = mobility->getId();
        // mobilityId = mobilityPtr->getExternalId();
        rootPtr = modulePtr->getParentModule();
        // EV << "Rootptr->getFullName() is " << rootPtr->getFullName() << std::endl;
        traci = mobility->getCommandInterface();
        vehicleCmdId = mobility->getVehicleCommandInterface();
        roadId = vehicleCmdId->getRoadId();
    }
    else if (stage == 1) {
        // Initializing members that require initialized other modules goes here
        // globalStats = veins::FindModule<GlobalStatistics*>::findSubModule(getParentModule()->getParentModule());
    }
    setTimer();
}

InterVehicleMessage* InteractingVehicle::generateMessage(){
    InterVehicleMessage* msg = new InterVehicleMessage();
    DemoBaseApplLayer::handlePositionUpdate(mobility);
    msg->setVehicleId(vehicleId);
    msg->setPosition(mobility->getPositionAt(simTime()));
    msg->setSpeed(mobility->getSpeed());
    return msg;
}

void InteractingVehicle::setTimer(){
    InterVehicleMessage* msg = InteractingVehicle::generateMessage();
    scheduleAt(simTime() + interval + uniform(0.01, 0.2), msg);
    return;
}

void InteractingVehicle::handleMessage(cMessage* msg)
{
    // handle self messages (e.g. timers) in a separate methods
    if (msg->isSelfMessage()) {
        handleSelfMsg(msg);
    }
    // put handling of messages from other nodes here
    else{
        if(InterVehicleMessage* iMsg = dynamic_cast<InterVehicleMessage*>(msg)){
            // cast success
            EV << "Message from: VehicleId -> " << iMsg->getVehicleId() << " , Position -> " << iMsg->getPosition() << ", and Speed -> " << iMsg->getSpeed();
        }
        else{
            // cast not success
            InterVehicleMessage* nMsg = InteractingVehicle::generateMessage();
            EV << "Cast fails!" << std::endl;
        }
    }
}

void InteractingVehicle::finish()
{
    DemoBaseApplLayer::finish();
    // maybe you want to record some scalars?
}

void InteractingVehicle::handleSelfMsg(cMessage* msg)
{
    if(InterVehicleMessage *iMsg = dynamic_cast<InterVehicleMessage*>(msg)){
        iMsg->setChannelNumber(static_cast<int>(veins::Channel::cch));
        sendDown(iMsg->dup());
        setTimer();
    }
    else
        DemoBaseApplLayer::handleSelfMsg(msg);
    // this method is for self messages (mostly timers)
    // it is important to call the DemoBaseApplLayer function for BSM and WSM transmission
}
