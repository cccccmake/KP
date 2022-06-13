//
// Generated file, do not edit! Created by nedtool 5.6 from InterVehicleMessage.msg.
//

#ifndef __INTERVEHICLEMESSAGE_M_H
#define __INTERVEHICLEMESSAGE_M_H

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0506
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



// cplusplus {{
	#include "veins/modules/messages/DemoSafetyMessage_m.h"
	#include "veins/base/utils/Coord.h"
// }}

/**
 * Class generated from <tt>InterVehicleMessage.msg:25</tt> by nedtool.
 * <pre>
 * // This is an example for a message type that is compatible to
 * // Veins simulations, as simple cMessage messages will cause exceptions.
 * packet InterVehicleMessage extends veins::DemoSafetyMessage
 * {
 *     // added vehicleId to identify vehicles
 *     int vehicleId;
 *     string roadId;
 *     veins::Coord position;
 *     // added speed into the package
 *     veins::Coord speed;
 * }
 * </pre>
 */
class InterVehicleMessage : public ::veins::DemoSafetyMessage
{
  protected:
    int vehicleId;
    ::omnetpp::opp_string roadId;
    veins::Coord position;
    veins::Coord speed;

  private:
    void copy(const InterVehicleMessage& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const InterVehicleMessage&);

  public:
    InterVehicleMessage(const char *name=nullptr, short kind=0);
    InterVehicleMessage(const InterVehicleMessage& other);
    virtual ~InterVehicleMessage();
    InterVehicleMessage& operator=(const InterVehicleMessage& other);
    virtual InterVehicleMessage *dup() const override {return new InterVehicleMessage(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual int getVehicleId() const;
    virtual void setVehicleId(int vehicleId);
    virtual const char * getRoadId() const;
    virtual void setRoadId(const char * roadId);
    virtual veins::Coord& getPosition();
    virtual const veins::Coord& getPosition() const {return const_cast<InterVehicleMessage*>(this)->getPosition();}
    virtual void setPosition(const veins::Coord& position);
    virtual veins::Coord& getSpeed();
    virtual const veins::Coord& getSpeed() const {return const_cast<InterVehicleMessage*>(this)->getSpeed();}
    virtual void setSpeed(const veins::Coord& speed);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const InterVehicleMessage& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, InterVehicleMessage& obj) {obj.parsimUnpack(b);}


#endif // ifndef __INTERVEHICLEMESSAGE_M_H

