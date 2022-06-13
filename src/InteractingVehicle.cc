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
            EV << "Received by " << vehicleId << ", speed, position: " << mobility->getCurrentDirection() * mobility->getSpeed() << " | " << mobility->getPositionAt(simTime()) << ", Message from: VehicleId -> " << iMsg->getVehicleId() << " , Position -> " << iMsg->getPosition() << ", and Speed -> " << iMsg->getSpeed() << std::endl;
            otherVehicleId = iMsg->getVehicleId();
            // Start of the task5
            /* Algorithm1:
             * calculate the intersection point
             * step1: record the position and speed vectors data from message. And some variables declarations
             * step2: compute the intersection point with the help of 3D-linear equations of two line( vehicle pair )
             * step3: compute the intersection time
             * */

            /* Algorithm2:
             * basic idea of the algorithm2: for two vehicles with position and velocity information
             *       Car1(x1,y1,z1) | V1(a1,b1,c1)  and Car1(x2,y2,z2) | V2(a2,b2,c2)
             * Let's say, they will meet somewhere at a certain time. Which means their final position will be the same.
             * and the time cost from the computing point until they meet will also be the same.
             * we can compute the time cost for each direction and they should be the same.
             * if the time cost for each direction are not the same value, it means those two cars will not meet or crash.
             *
             * step1: compute the time cost of each direction
             * step2: compute the intersection point
             * */
            // initialize the time cost for each direction
            double tX = INFINIT, tY = INFINIT, tZ = INFINIT;
            // variables used to record the result.
            veins::Coord resPosition;
            double resTime = 0;
            // get position and speed information of other's car and my car.
            veins::Coord positionFromMessage = iMsg->getPosition();
            veins::Coord speedFromMessage = iMsg->getSpeed();
            myCoord = mobility->getPositionAt(simTime());
            mySpeed = mobility->getCurrentDirection() * mobility->getSpeed();

            //step1
            if(veins::math::almost_equal(mySpeed.x, speedFromMessage.x));
            else tX = (positionFromMessage.x - myCoord.x) / (mySpeed.x - speedFromMessage.x);
            if(veins::math::almost_equal(mySpeed.y, speedFromMessage.y));
            else tY = (positionFromMessage.y - myCoord.y) / (mySpeed.y - speedFromMessage.y);
            if(veins::math::almost_equal(mySpeed.z, speedFromMessage.z));
            else tZ = (positionFromMessage.z - myCoord.z) / (mySpeed.z - speedFromMessage.z);

            // if it is a 2D plane, then we only need to consider X and Y
            if(myCoord.z == positionFromMessage.z && mySpeed.z == speedFromMessage.z){
                tZ = 0;
                resPosition.z = 0;
                if(tX < 0 || tY < 0) EV << "No intersection" << std::endl;
                else if(tX < INFINIT
                        && veins::math::almost_equal(tX, tY))
                {
                    resTime = tX;
                    resPosition.x = myCoord.x + resTime * mySpeed.x;
                    resPosition.y = myCoord.y + resTime * mySpeed.y;
                    intersectionTimeRecord.insert({otherVehicleId, simTime().dbl() + resTime});
                    intersectionPointRecord.insert({otherVehicleId, resPosition});
                }
                else EV << "No intersection" << std::endl;
            }
            else{// if it is in a 3D space, direction Z is in consideration.
                if(tX < 0 || tY < 0 || tZ <0) EV << "No intersection" << std::endl;
                else if(tX < INFINIT
                        && veins::math::almost_equal(tX, tY)
                        && veins::math::almost_equal(tX, tZ)){
                    resTime = tX;
                    resPosition.x = myCoord.x + resTime * mySpeed.x;
                    resPosition.y = myCoord.y + resTime * mySpeed.y;
                    resPosition.z = myCoord.z + resTime * mySpeed.z;
                    intersectionTimeRecord.insert({otherVehicleId, simTime().dbl() + resTime});
                    intersectionPointRecord.insert({otherVehicleId, resPosition});
                }
                else{
                    EV << "No intersection" << std::endl;
                }
            }
            EV << "tX: " << tX <<  "tY: " << tY << "tZ: " << tZ << std::endl;
            EV << "time: " << resTime <<  " and position: " << resPosition << std::endl;
            // end of task 5
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
