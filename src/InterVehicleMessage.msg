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

cplusplus {{
	#include "veins/modules/messages/DemoSafetyMessage_m.h"
	#include "veins/base/utils/Coord.h"
}}
class veins::DemoSafetyMessage;
class noncobject veins::Coord;

// This is an example for a message type that is compatible to
// Veins simulations, as simple cMessage messages will cause exceptions.
packet InterVehicleMessage extends veins::DemoSafetyMessage {
    // added vehicleId to identify vehicles
	int vehicleId;
	veins::Coord position;
	// added speed into the package
	double speed;
}
