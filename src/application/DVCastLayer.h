//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef DVCastLayer_H
#define DVCastLayer_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "messages/DVCast_m.h"

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;

/**
 * Small IVC Demo using 11p
 */
class DVCastLayer: public BaseWaveApplLayer {
public:
    virtual void initialize(int stage);
    virtual void receiveSignal(cComponent* source, simsignal_t signalID,
            cObject* obj, cObject* details);
protected:
    TraCIMobility* mobility;
    TraCICommandInterface* traci;
    TraCICommandInterface::Vehicle* traciVehicle;
    AnnotationManager* annotations;
    simtime_t lastDroveAt;

    double lasty = 0;
    double lastx = 0;
    double dlasty = 0;
    double dlastx = 0;
    std::deque<int> NB_FRONT, NB_BACK, NB_OPPOSITE;
    std::deque<int> rcvdMessages;
    std::map<int, WaveShortMessage*> delayedRB;
    bool sentAccidentMessage;
    bool isParking;
    bool sendWhileParking;
    static const simsignalwrap_t parkingStateChangedSignal;
protected:
    virtual void onBeacon(WaveShortMessage* wsm);
    virtual void onData(WaveShortMessage* wsm);
    virtual void handlePositionUpdate(cObject* obj);
    virtual void handleParkingUpdate(cObject* obj);
    virtual void handleLowerMsg(cMessage* msg);
    virtual void sendWSM(WaveShortMessage* wsm);

    void onHello(DVCast* wsm);
    void neigbors_tables(Coord senderPosition, int senderId, int senderAngle);
    void sendHello(DVCast* wsm);
    void sendMessage(std::string blockedRoadId, int recipient, int serial);
    DVCast* prepareHello(std::string name, int lengthBits, t_channel channel,
            int priority, int rcvId, int serial);
public:


};

template<class InputIterator, class T>
static InputIterator find(InputIterator first, InputIterator last, const T& val) {
        while (first != last) {
            if (*first == val)
                return first;
            ++first;
        }
        return last;
    }
#endif
