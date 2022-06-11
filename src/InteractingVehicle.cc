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
        // omnetpp::cModule* globalStats = veins::FindModule<GlobalStatistics*>::findSubModule(getParentModule()->getParentModule());
    }
    setTimer();
}

InterVehicleMessage* InteractingVehicle::generateMessage(){
    InterVehicleMessage* msg = new InterVehicleMessage();
    // DemoBaseApplLayer::handlePositionUpdate(mobility);
    // seems the invocation of this method is not necessary.
    msg->setVehicleId(vehicleId);
    msg->setPosition(mobility->getPositionAt(simTime()));
    msg->setSpeed(mobility->getCurrentDirection() * mobility->getSpeed());
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
            EV << "Message from: VehicleId -> " << iMsg->getVehicleId() << " , Position -> " << iMsg->getPosition() << ", and Speed -> " << iMsg->getSpeed() << std::endl;
            // initial value of the intersection time
            double tX = INFINIT, tY = INFINIT;
            // store result
            veins::Coord resPosition;
            double resTime = 0;
            double positionX = iMsg->getPosition().x;
            double positionY = iMsg->getPosition().y;
            double speedX = iMsg->getSpeed().x;
            double speedY = iMsg->getSpeed().y;
            myCoord = mobility->getPositionAt(simTime());
            mySpeed = mobility->getCurrentDirection() * mobility->getSpeed();

            if(veins::math::almost_equal(mySpeed.x, speedX))
                // no need to process
                ;
            else
                double tX = (positionX - myCoord.x) / (mySpeed.x - speedX);

            if(veins::math::almost_equal(mySpeed.y, speedY))
                ;
            else
                double tY = (positionY - myCoord.y) / (mySpeed.y - speedY);

            if(tX <= tY)
                resTime = tY;
            else
                resTime = tX;

            if(resTime != INFINIT){
                resPosition.x = myCoord.x + resTime * mySpeed.x;
                resPosition.y = myCoord.y + resTime * mySpeed.y;
            }
            else{
                ;
            }

            EV << "time: " << resTime <<  " and position: " << resPosition << std::endl;
            intersectionTimeRecord.insert({vehicleId, simTime() + resTime});
            intersectionPointRecord.insert({vehicleId, resPosition});

        }
        else{
            // cast not success
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
