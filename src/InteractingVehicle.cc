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
        // modulePtr = veins::TraCIMobilityAccess().get(getParentModule())->getParentModule();
        // EV << modulePtr->getFullName(); // node[i]
        vehicleId = modulePtr->getId();
        vehicleIndex = modulePtr->getIndex();
        // vehicleId, refer to the log left side on simulation
        // vehicleIndex is the index of the node in the vector.
        mobilityPtr = veins::TraCIMobilityAccess().get(getParentModule());
        mobilityId = mobilityPtr->getId();
        rootPtr = modulePtr->getParentModule();
        // EV << "Rootptr->getFullName() is " << rootPtr->getFullName() << std::endl;
        // EV << "trying to get submodule: " << mobilityPtr->getSubmodule("node") << std::endl;
        traci = mobility->getCommandInterface();
        vehicleCmdId = mobility->getVehicleCommandInterface();
        roadId = vehicleCmdId->getRoadId();
        /* DemoBaseApplLayer::handlePositionUpdate(mobilityPtr); */
        /* Must invoke first. Otherwise the value 0. */
        /* seems dynamic_cast<cObject*>(mobilityPtr) not necessary */
    }
    else if (stage == 1) {
        // Initializing members that require initialized other modules goes here
        // globalStats = veins::FindModule<GlobalStatistics*>::findSubModule(getParentModule()->getParentModule());
    }
    setTimer();
}

InterVehicleMessage* InteractingVehicle::generateMessage(){
    InterVehicleMessage *msg = new InterVehicleMessage();
    DemoBaseApplLayer::handlePositionUpdate(mobilityPtr);
    msg->setVehicleId(vehicleId);
    msg->setPosition(curPosition);
    msg->setSpeed(curSpeed);
    return msg;
}

void InteractingVehicle::setTimer(){
    InterVehicleMessage* msg = InteractingVehicle::generateMessage();
    scheduleAt(simTime() + 2 + uniform(0.01, 0.2), msg);
    return;
}

void InteractingVehicle::handleMessage(cMessage* msg)
{
    // handle self messages (e.g. timers) in a separate methods
    if (msg->isSelfMessage()) {
        // I added some code in DemoBaseApplLayer.cc regarding to the schedule conflict.
        handleSelfMsg(msg);
    }
    // put handling of messages from other nodes here
    else{
        EV << "In handling other MSG" << std::endl;
        InterVehicleMessage* iMsg = dynamic_cast<InterVehicleMessage*>(msg);
        if(iMsg != nullptr){
            // cast success
            EV << "pos: " << iMsg->getPosition() << " and speed: " << iMsg->getSpeed();
        }
        else{
            // cast not success
            EV << "msg type: " << msg->getClassName() << std::endl;
            // MSG type: DemoSafetyMessage
            EV << "Due to casting failure, handling Messages from other vehicles not possible" << std::endl;
            InterVehicleMessage* nMsg = InteractingVehicle::generateMessage();
            EV << "Sending new message, id, position, speed: " << nMsg->getVehicleId() << " " << nMsg->getPosition() << " " << nMsg->getSpeed() << std::endl;
            sendDown(nMsg);
        }
    }
    setTimer();
}

void InteractingVehicle::finish()
{
    DemoBaseApplLayer::finish();
    // maybe you want to record some scalars?
}

void InteractingVehicle::handleSelfMsg(cMessage* msg)
{
    DemoBaseApplLayer::handleSelfMsg(msg);
    // this method is for self messages (mostly timers)
    // it is important to call the DemoBaseApplLayer function for BSM and WSM transmission
}

