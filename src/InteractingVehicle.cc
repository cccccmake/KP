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
        givenTime = par("givenTime");
        threshold = par("threshold");
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
    msg->setRoadId(roadId.c_str());
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
            EV << "Received by " << vehicleId << ", speed, position: " << mobility->getCurrentDirection() * mobility->getSpeed() << " | " << mobility->getPositionAt(simTime()) << ", Message from: VehicleId -> " << iMsg->getVehicleId() << " , Position -> " << iMsg->getPosition() << ", Speed -> " << iMsg->getSpeed() << " and roadId -> " << iMsg->getRoadId() << std::endl;
            otherVehicleId = iMsg->getVehicleId();
            // Start of the task5
            /* Algorithm1:
             * calculate the intersection point
             * step1: record the position and speed vectors data from message. And some variables declarations
             * step2: compute the intersection point with the help of 3D-linear equations of two line( vehicle pair )
             * step3: compute the intersection time
             * */
            ///////////////////////////////// Algorithm1 //////////////////////////////////////////////////////
                //step1
            veins::Coord resPosition;
            double myTime, hisTime; // record the time of arriving at the intersection point
            bool intersect = false;
            veins::Coord positionFromMessage = iMsg->getPosition();
            veins::Coord speedFromMessage = iMsg->getSpeed();
            myCoord = mobility->getPositionAt(simTime());
            mySpeed = mobility->getCurrentDirection() * mobility->getSpeed();
                //step2
            if(myCoord.z == positionFromMessage.z && mySpeed.z == speedFromMessage.z){
                // if it is a 2D plane, then we only need to consider X and Y
                if (mySpeed.x == 0 && mySpeed.y != 0 &&
                        speedFromMessage.x != 0 && speedFromMessage.y != 0){
                    resPosition.x = myCoord.x;
                    resPosition.y = speedFromMessage.y * ( myCoord.x - positionFromMessage.x ) / speedFromMessage.x + positionFromMessage.y;
                    intersect = true;
                }
                else if (mySpeed.y == 0 && mySpeed.x != 0 &&
                        speedFromMessage.x != 0 && speedFromMessage.y != 0){
                    resPosition.y = myCoord.y;
                    resPosition.x = speedFromMessage.x * ( myCoord.y - positionFromMessage.y ) / speedFromMessage.y + positionFromMessage.x;
                    intersect = true;
                }
                else if (speedFromMessage.x == 0 && mySpeed.y != 0 &&
                        mySpeed.x != 0 && speedFromMessage.y != 0){
                    resPosition.x = positionFromMessage.x;
                    resPosition.y = mySpeed.y * ( positionFromMessage.x - myCoord.x ) / mySpeed.x + myCoord.y;
                    intersect = true;
                }
                else if (speedFromMessage.y == 0 && mySpeed.y != 0 &&
                        mySpeed.x != 0 && speedFromMessage.x != 0){
                    resPosition.y = positionFromMessage.y;
                    resPosition.x = mySpeed.x * ( positionFromMessage.y - myCoord.y ) / mySpeed.y + myCoord.x;
                    intersect = true;
                }
                else if(mySpeed.x != 0 && mySpeed.y != 0 &&
                   speedFromMessage.x != 0 && speedFromMessage.y != 0){
                    //if the velocity of vehicle pair are not 0, we can use a point and a direction vector to represent a line
                    if (veins::math::almost_equal(speedFromMessage.x * mySpeed.y, mySpeed.x * speedFromMessage.y)){
                        EV << "They are on the same direction" << std::endl;
                    }
                    else{
                        resPosition.x = (speedFromMessage.x * mySpeed.y * myCoord.x - mySpeed.x * speedFromMessage.y * positionFromMessage.x
                                    - mySpeed.x * speedFromMessage.x * ( myCoord.y - positionFromMessage.y)) / (mySpeed.y * speedFromMessage.x -
                                            mySpeed.x * speedFromMessage.y);
                        resPosition.y = mySpeed.y / mySpeed.x * ( resPosition.x - myCoord.x ) + myCoord.y;
                        intersect = true;
                    }
                }
                else {
                    intersect = false;
                    EV << "No intersection" << std::endl;
                }

                //step3
                if (intersect){
                    myTime = ( resPosition.x - myCoord.x ) / mySpeed.x;
                    hisTime = ( resPosition.x - positionFromMessage.x ) / speedFromMessage.x;
                    if ( myTime < 0 || hisTime < 0){
                        EV << "They will not encounter" << std::endl;
                    }
                    else{
                        EV << "my time: " << myTime << "his time: " << hisTime << " and position: " << resPosition << std::endl;
                        visualization_TimeDiff(myTime, hisTime, threshold);
                        visualization_GivenTime(myTime, givenTime);
                        visualization_Brake(mobility, iMsg, myTime, hisTime, threshold);
                    }
                }
            }
            else{
                //if it is in a 3D space, direction Z is in consideration.
                if (mySpeed.x == 0 && mySpeed.y != 0 && mySpeed.z != 0 &&
                    speedFromMessage.x != 0 && speedFromMessage.y != 0 && speedFromMessage.z != 0){
                    resPosition.x = myCoord.x;
                    resPosition.y = positionFromMessage.y + speedFromMessage.y * (myCoord.x - positionFromMessage.x) / speedFromMessage.x;
                    resPosition.z = positionFromMessage.z + speedFromMessage.z * (myCoord.x - positionFromMessage.x) / speedFromMessage.x;
                    intersect = true;
                }
                else if (mySpeed.y == 0 && mySpeed.x != 0 && mySpeed.z != 0 &&
                         speedFromMessage.x != 0 && speedFromMessage.y != 0 && speedFromMessage.z != 0){
                    resPosition.y = myCoord.y;
                    resPosition.x = positionFromMessage.x + speedFromMessage.x * (myCoord.y - positionFromMessage.y) / speedFromMessage.y;
                    resPosition.z = positionFromMessage.z + speedFromMessage.z * (myCoord.y - positionFromMessage.y) / speedFromMessage.y;
                    intersect = true;
                }
                else if (mySpeed.z == 0 && mySpeed.y != 0 && mySpeed.x != 0 &&
                         speedFromMessage.x != 0 && speedFromMessage.y != 0 && speedFromMessage.z != 0){
                    resPosition.z = myCoord.z;
                    resPosition.x = positionFromMessage.x + speedFromMessage.x * (myCoord.z - positionFromMessage.z) / speedFromMessage.z;
                    resPosition.y = positionFromMessage.y + speedFromMessage.y * (myCoord.z - positionFromMessage.z) / speedFromMessage.z;
                    intersect = true;
                }
                else if (speedFromMessage.x == 0 && speedFromMessage.y != 0 && speedFromMessage.z != 0 &&
                         mySpeed.z != 0 && mySpeed.y != 0 && mySpeed.x != 0){
                    resPosition.x = positionFromMessage.x;
                    resPosition.y = myCoord.y + mySpeed.y * (positionFromMessage.x - myCoord.x) / mySpeed.x;
                    resPosition.z = myCoord.z + mySpeed.z * (positionFromMessage.x - myCoord.x) / mySpeed.x;
                    intersect = true;
                }
                else if (speedFromMessage.y == 0 && speedFromMessage.x != 0 && speedFromMessage.z != 0 &&
                         mySpeed.z != 0 && mySpeed.y != 0 && mySpeed.x != 0){
                    resPosition.y = positionFromMessage.y;
                    resPosition.x = myCoord.x + mySpeed.x * (positionFromMessage.y - myCoord.y) / mySpeed.y;
                    resPosition.z = myCoord.z + mySpeed.z * (positionFromMessage.y - myCoord.y) / mySpeed.y;
                    intersect = true;
                }
                else if (speedFromMessage.z == 0 && speedFromMessage.x != 0 && speedFromMessage.x != 0 &&
                         mySpeed.z != 0 && mySpeed.y != 0 && mySpeed.x != 0){
                    resPosition.z = positionFromMessage.z;
                    resPosition.x = myCoord.x + mySpeed.x * (positionFromMessage.z - myCoord.z) / mySpeed.z;
                    resPosition.z = myCoord.z + mySpeed.z * (positionFromMessage.z - myCoord.z) / mySpeed.z;
                    intersect = true;
                }
                else if (mySpeed.x == 0 && mySpeed.y == 0 && mySpeed.z != 0 &&
                         speedFromMessage.x != 0 && speedFromMessage.y != 0 && speedFromMessage.z != 0){
                    resPosition.x = myCoord.x;
                    resPosition.y = myCoord.y;
                    resPosition.z = positionFromMessage.z + speedFromMessage.z * (myCoord.x - positionFromMessage.x) / speedFromMessage.x;
                    intersect = true;
                }
                else if (mySpeed.x == 0 && mySpeed.z == 0 && mySpeed.y != 0 &&
                         speedFromMessage.x != 0 && speedFromMessage.y != 0 && speedFromMessage.z != 0){
                    resPosition.x = myCoord.x;
                    resPosition.z = myCoord.z;
                    resPosition.y = positionFromMessage.y + speedFromMessage.y * (myCoord.x - positionFromMessage.x) / speedFromMessage.x;
                    intersect = true;
                }
                else if (mySpeed.y == 0 && mySpeed.z == 0 && mySpeed.x != 0 &&
                         speedFromMessage.x != 0 && speedFromMessage.y != 0 && speedFromMessage.z != 0){
                    resPosition.y = myCoord.y;
                    resPosition.z = myCoord.z;
                    resPosition.x = positionFromMessage.x + speedFromMessage.x * (myCoord.y - positionFromMessage.y) / speedFromMessage.y;
                    intersect = true;
                }
                else if (speedFromMessage.x == 0 && speedFromMessage.y == 0 && speedFromMessage.z != 0 &&
                         mySpeed.z != 0 && mySpeed.y != 0 && mySpeed.x != 0){
                    resPosition.x = positionFromMessage.x;
                    resPosition.y = positionFromMessage.y;
                    resPosition.z = myCoord.z + mySpeed.z * (positionFromMessage.x - myCoord.x) / mySpeed.x;
                    intersect = true;
                }
                else if (speedFromMessage.x == 0 && speedFromMessage.z == 0 && speedFromMessage.y != 0 &&
                         mySpeed.z != 0 && mySpeed.y != 0 && mySpeed.x != 0){
                    resPosition.x = positionFromMessage.x;
                    resPosition.z = positionFromMessage.z;
                    resPosition.y = myCoord.y + mySpeed.y * (positionFromMessage.x - myCoord.x) / mySpeed.x;
                    intersect = true;
                }
                else if (speedFromMessage.z == 0 && speedFromMessage.y == 0 && speedFromMessage.x != 0 &&
                         mySpeed.z != 0 && mySpeed.y != 0 && mySpeed.x != 0){
                    resPosition.y = positionFromMessage.y;
                    resPosition.z = positionFromMessage.z;
                    resPosition.x = myCoord.x + mySpeed.x * (positionFromMessage.z - myCoord.z) / mySpeed.z;
                    intersect = true;
                }
                else if (speedFromMessage.z != 0 && speedFromMessage.y != 0 && speedFromMessage.x != 0 &&
                         mySpeed.z != 0 && mySpeed.y != 0 && mySpeed.x != 0){
                    resPosition.x = (myCoord.x * speedFromMessage.x - positionFromMessage.x * mySpeed.x) / (speedFromMessage.x - mySpeed.x);
                    resPosition.y = (myCoord.y * speedFromMessage.y - positionFromMessage.y * mySpeed.y) / (speedFromMessage.y - mySpeed.y);
                    resPosition.z = (myCoord.z * speedFromMessage.z - positionFromMessage.z * mySpeed.z) / (speedFromMessage.z - mySpeed.z);
                    intersect = true;
                }
                else{
                    intersect = false;
                    EV << "No intersection" << std::endl;
                }

                if (intersect){
                    myTime = ( resPosition.x - myCoord.x ) / mySpeed.x;
                    hisTime = ( resPosition.x - positionFromMessage.x ) / speedFromMessage.x;
                    if ( myTime < 0 || hisTime < 0){
                        EV << "No intersection" << std::endl;
                    }
                    else{
                        EV << "my time: " << myTime << "his time: " << hisTime << " and position: " << resPosition << std::endl;
                        visualization_TimeDiff(myTime, hisTime, threshold);
                        visualization_GivenTime(myTime, givenTime);
                        visualization_Brake(mobility, iMsg, myTime, hisTime, threshold);
                    }
                }
            }
            ///////////////////////////////////////// Algorithm1 ////////////////////////////////////////////////

            ///////////////////////////////////////// Algorithm2 ////////////////////////////////////////////////
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
//            // initialize the time cost for each direction
//            double tX = INFINIT, tY = INFINIT, tZ = INFINIT;
//            // variables used to record the result.
//            veins::Coord resPosition;
//            double resTime = 0;
//            // get position and speed information of other's car and my car.
//            veins::Coord positionFromMessage = iMsg->getPosition();
//            veins::Coord speedFromMessage = iMsg->getSpeed();
//            myCoord = mobility->getPositionAt(simTime());
//            mySpeed = mobility->getCurrentDirection() * mobility->getSpeed();
//
//            //step1
//            if(veins::math::almost_equal(mySpeed.x, speedFromMessage.x));
//            else tX = (positionFromMessage.x - myCoord.x) / (mySpeed.x - speedFromMessage.x);
//            if(veins::math::almost_equal(mySpeed.y, speedFromMessage.y));
//            else tY = (positionFromMessage.y - myCoord.y) / (mySpeed.y - speedFromMessage.y);
//            if(veins::math::almost_equal(mySpeed.z, speedFromMessage.z));
//            else tZ = (positionFromMessage.z - myCoord.z) / (mySpeed.z - speedFromMessage.z);
//
//            // if it is a 2D plane, then we only need to consider X and Y
//            if(myCoord.z == positionFromMessage.z && mySpeed.z == speedFromMessage.z){
//                tZ = 0;
//                resPosition.z = 0;
//                if(tX < 0 || tY < 0) EV << "No intersection" << std::endl;
//                else if(tX < INFINIT
//                        && veins::math::almost_equal(tX, tY))
//                {
//                    resTime = tX;
//                    resPosition.x = myCoord.x + resTime * mySpeed.x;
//                    resPosition.y = myCoord.y + resTime * mySpeed.y;
//                    intersectionTimeRecord.insert({otherVehicleId, simTime().dbl() + resTime});
//                    intersectionPointRecord.insert({otherVehicleId, resPosition});
//                }
//                else EV << "No intersection" << std::endl;
//            }
//            else{// if it is in a 3D space, direction Z is in consideration.
//                if(tX < 0 || tY < 0 || tZ <0) EV << "No intersection" << std::endl;
//                else if(tX < INFINIT
//                        && veins::math::almost_equal(tX, tY)
//                        && veins::math::almost_equal(tX, tZ)){
//                    resTime = tX;
//                    resPosition.x = myCoord.x + resTime * mySpeed.x;
//                    resPosition.y = myCoord.y + resTime * mySpeed.y;
//                    resPosition.z = myCoord.z + resTime * mySpeed.z;
//                    intersectionTimeRecord.insert({otherVehicleId, simTime().dbl() + resTime});
//                    intersectionPointRecord.insert({otherVehicleId, resPosition});
//                }
//                else{
//                    EV << "No intersection" << std::endl;
//                }
//            }
//            EV << "tX: " << tX <<  "tY: " << tY << "tZ: " << tZ << std::endl;
//            EV << "time: " << resTime <<  " and position: " << resPosition << std::endl;
//            // end of task 5
//        }
//        else{
//            // cast not success
//            EV << "Cast fails!" << std::endl;

        ///////////////////////////////////// Algorithm2 //////////////////////////////////////////////////

        }
    }
}

void InteractingVehicle::visualization_TimeDiff(double myTime, double hisTime, double threshold){
    if(myTime > 0 && hisTime > 0 && std::abs(myTime - hisTime) < threshold)
        getParentModule()->bubble("The time difference is below the threshold!");
}

void InteractingVehicle::visualization_GivenTime(double myTime, double givenTime){
    if(myTime > 0 && myTime < givenTime){
        std::string init("The potential collision is below the given time! Estimated time is: ");
        std::string toAppend(std::to_string(myTime));
        const char * text = (init.append(toAppend).c_str());
        getParentModule()->bubble(text);
    }
}

void InteractingVehicle::visualization_Brake(veins::TraCIMobility* mobility, InterVehicleMessage* msg, double myTime, double hisTime, double threshold){
    const char* tempRoadId = msg->getRoadId();
    std::string hisRoadId = tempRoadId;
    if(mobility->getRoadId() == "-gneE1" && hisRoadId == "-gneE2"){
        double currentSpeed = mobility->getSpeed();
        if(myTime > 0 && myTime < threshold){
            DemoBaseApplLayer::traciVehicle->setSpeed(0);
        }
        else if(hisTime < 0){
            DemoBaseApplLayer::traciVehicle->setSpeed(currentSpeed);
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
