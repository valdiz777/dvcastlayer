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

#include "DVCastLayer.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t DVCastLayer::parkingStateChangedSignal = simsignalwrap_t(
TRACI_SIGNAL_PARKING_CHANGE_NAME);

// control when to send heartbeat
bool sendHeartBeat;

Define_Module(DVCastLayer);
void DVCastLayer::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        EV << "****initialize" << endl;
        mobility = TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        sentMessage = false;
        lastDroveAt = simTime();
        findHost()->subscribe(parkingStateChangedSignal, this);
        isParking = false;
        sendWhileParking = par("sendWhileParking").boolValue();

        EV << "Car [" << getParentModule()->getIndex()
                  << "] Entering CPE646 DVCast Simulation " << endl;
    }
}

void DVCastLayer::onBeacon(WaveShortMessage* wsm) {
}

// received accident data, process dv-cast
void DVCastLayer::onData(WaveShortMessage* wsm) {
    EV << "****onData" << endl;
    findHost()->getDisplayString().updateWith("r=16,green");
    annotations->scheduleErase(1,
            annotations->drawLine(wsm->getSenderPos(),
                    mobility->getPositionAt(simTime()), "blue"));

    if (mobility->getRoadId()[0] != ':')
        traciVehicle->changeRoute(wsm->getWsmData(), 9999);
    if (!sentMessage) /*** start here to process dvcast and do broadcast suppression**/
        sendMessage(wsm->getWsmData());//right now no broadcast suppression
}

void DVCastLayer::sendMessage(std::string blockedRoadId) {
    EV << "****sendMessage" << endl;
    sentMessage = true; //won't be needed after we switch to dv-cast

    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel,
            dataPriority, -1, 2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
}

void DVCastLayer::receiveSignal(cComponent* source, simsignal_t signalID,
        cObject* obj, cObject* details) {
    EV << "****receiveSignal" << endl;
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    } else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void DVCastLayer::handleParkingUpdate(cObject* obj) {
    EV << "****handleParkingUpdate" << endl;
    isParking = mobility->getParkingState();
    if (sendWhileParking == false) {
        if (isParking == true) {
            (FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(
                    this->getParentModule()->getSubmodule("nic"));
        } else {
            Coord pos = mobility->getCurrentPosition();
            (FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(
                    this->getParentModule()->getSubmodule("nic"),
                    (ChannelAccess*) this->getParentModule()->getSubmodule(
                            "nic")->getSubmodule("phy80211p"), &pos);
        }
    }
}

// send heartbeats evertime we move
void DVCastLayer::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);

    dlastx += std::abs(mobility->getCurrentPosition().x - lastx);
    dlasty += std::abs(mobility->getCurrentPosition().y - lasty);

    if ((dlastx >= 50) || (dlasty >= 50)) {
        DVCast* wsm = preparemyWSM("heartbeat", beaconLengthBits, type_CCH,
                beaconPriority, 0, 2);
        send_WSM(wsm);
        dlastx = (dlastx > 50) ? 0 : dlastx;
        dlasty = (dlasty > 50) ? 0 : dlasty;
    }

    if (mobility->getSpeed() < 1) {
        // stopped for for at least 10s?
        if (simTime() - lastDroveAt >= 10) {
            findHost()->getDisplayString().updateWith("r=16,red");
            if (!sentMessage)
                sendMessage(mobility->getRoadId());
        }
    } else {
        lastDroveAt = simTime();
    }
    lastx = mobility->getCurrentPosition().x;
    lasty = mobility->getCurrentPosition().y;
}

void TraCIDemo11p::sendWSM(WaveShortMessage* wsm) {
    if (isParking && !sendWhileParking) return;
    sendDelayedDown(wsm,individualOffset);
}

void DVCastLayer::send_WSM(DVCast* wsm) {

    if (isParking && !sendWhileParking)
        return;
    EV << "****send_WSM" << endl;
    sendDelayedDown(wsm, individualOffset);
}

// route messages manually
void DVCastLayer::handleLowerMsg(cMessage* msg) {

    DVCast* wsm = dynamic_cast<DVCast*>(msg);
    ASSERT(wsm);
    EV << "****handleLowerMsg" << endl;
    if (std::string(wsm->getName()) == "data") {
        onData(wsm);
    } else if (std::string(wsm->getName()) == "heartbeat") {
        onHeartBeatReceived(wsm);
    } else {
        DBG << "unknown message (" << wsm->getName() << ")  received\n";
    }
    delete (msg);
}

void DVCastLayer::onHeartBeatReceived(DVCast* wsm) {
    EV << "****onHeartBeatReceived" << endl;
    EV << "My Position x:" << mobility->getCurrentPosition().x
              << " My Position y:" << mobility->getCurrentPosition().y
              << " My Position z:" << mobility->getCurrentPosition().z
              << " My Id: " << getParentModule()->getIndex() << endl;
    EV << "Sender Position x:" << wsm->getSenderPos().x << " Sender Position y:"
              << wsm->getSenderPos().y << " Sender Position z:"
              << wsm->getSenderPos().z << " Sender id: "
              << wsm->getSenderAddress() << endl;
    if (mobility->getCurrentPosition().x < wsm->getRoi_up().x
            && mobility->getCurrentPosition().y < wsm->getRoi_up().y
            && mobility->getCurrentPosition().x > wsm->getRoi_down().x
            && mobility->getCurrentPosition().y > wsm->getRoi_down().y) {
        neigbors_tables(wsm->getSenderPos(), wsm->getSenderAddress());
    }
}

// dvcast flow chart
void DVCastLayer::onMyMessage(DVCast* wsm) {
    EV << "****onMyMessage" << endl;
    EV << "Recipient Address " << wsm->getRecipientAddress() << " My Id "
              << getParentModule()->getIndex() << endl;

    /*if (wsm->getRecipientAddress() == getParentModule()->getIndex()) { // DFlg equals 1
     EV << "The message reached its destination" << endl;
     } else {
     send_Message(wsm);
     }*/
}

void DVCastLayer::send_Message(DVCast* wsm) {
    EV << "****Send_Message" << endl;
    if (!NB_FRONT.empty()) {        // MDC equals 1
        EV << "I sent the message : " << NB_FRONT.top() << endl;
        wsm->setRecipientAddress(NB_FRONT.top());
        sendDelayedDown(wsm, individualOffset);
    } else {
        if (!NB_OPPOSITE.empty()) { // ODC equals 1
            EV << "I sent the message : " << NB_OPPOSITE.top() << endl;
            wsm->setRecipientAddress(NB_OPPOSITE.top());
            sendDelayedDown(wsm, individualOffset);
        } else {
            EV << "Send_Message No message sent" << endl;
        }
    }
}

//Main algorithm used to decide MDC, OPC, and Dflg
void DVCastLayer::neigbors_tables(Coord senderPosition, int senderId) {
    EV << "**********neigbors_tables Message has arrived MyId:"
              << getParentModule()->getIndex() << " SenderId:" << senderId
              << endl;
    /*EV << "My Position x:" << mobility->getCurrentPosition().x
     << " My Position y:" << mobility->getCurrentPosition().y
     << " My Position z:" << mobility->getCurrentPosition().z
     << " My Id: " << getParentModule()->getIndex() << endl;
     EV << "Sender Position x:" << wsm->getSenderPos().x << " Sender Position y:"
     << wsm->getSenderPos().y << " Sender Position z:"
     << wsm->getSenderPos().z << " Sender id: "
     << wsm->getSenderAddress() << endl;*/
    if (mobility->getCurrentPosition().y > 2000) {
        if (senderPosition.y < 6000) {
            EV << "ID" << getParentModule()->getIndex() << "NBOPPOSITE"
                      << senderId << endl;
            NB_OPPOSITE.push(senderId);
        } else {
            if (senderPosition.x < mobility->getCurrentPosition().x) {
                EV << "ID " << getParentModule()->getIndex() << " NBFRONT "
                          << senderId << endl;
                NB_FRONT.push(senderId);
            }
            if (senderPosition.x > mobility->getCurrentPosition().x) {
                EV << "ID" << getParentModule()->getIndex() << " NBBACK "
                          << senderId << endl;
                NB_BACK.push(senderId);
            }
        }
    } else {        // car is travelling through the right side
        if (senderPosition.y > 2000) {
            EV << "ID" << getParentModule()->getIndex() << " NBOPPOSITE "
                      << senderId << endl;
            if (NB_OPPOSITE.size() == 5) { // For ensuring five of stack size
                NB_OPPOSITE.pop();
            }
            NB_OPPOSITE.push(senderId);
        } else {
            if (senderPosition.x > mobility->getCurrentPosition().x) {
                EV << "ID " << getParentModule()->getIndex() << " NBFRONT "
                          << senderId << endl;
                if (NB_OPPOSITE.size() == 5) { // For ensuring five of stack size
                    NB_OPPOSITE.pop();
                }
                NB_FRONT.push(senderId);
            }
            if (senderPosition.x < mobility->getCurrentPosition().x) {
                EV << "ID" << getParentModule()->getIndex() << " NBBACK "
                          << senderId << endl;
                if (NB_OPPOSITE.size() == 5) { // For ensuring five of stack size
                    NB_OPPOSITE.pop();
                }
                NB_BACK.push(senderId);
            }
        }
    }
}

DVCast* DVCastLayer::preparemyWSM(std::string name, int lengthBits,
        t_channel channel, int priority, int rcvId, int serial) {
    EV << "****preparemyWSM" << endl;
    DVCast* wsm = new DVCast(name.c_str(), 0);
    wsm->addBitLength(headerLength);
    wsm->addBitLength(lengthBits);

    switch (channel) {
    case type_SCH:
        wsm->setChannelNumber(Channels::SCH1);
        break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
    case type_CCH:
        wsm->setChannelNumber(Channels::CCH);
        break;
    }
    wsm->setPsid(0);
    wsm->setPriority(priority);
    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSenderAddress(getParentModule()->getIndex());
    wsm->setSenderPos(curPosition);

    wsm->setSerial(serial);

    if (name == "beacon") {
        DBG << "Creating Beacon with Priority " << priority
                   << " at Applayer at " << wsm->getTimestamp() << std::endl;
    }
    if (name == "data") {
        DBG << "Creating Data with Priority " << priority << " at Applayer at "
                   << wsm->getTimestamp() << std::endl;
    }

    if (name == "heartbeat") {
        DBG << "Creating Heartbeat with Priority " << priority
                   << " at Applayer at " << wsm->getTimestamp() << std::endl;
        wsm->setRoi_up(Coord(curPosition.x + 50, curPosition.y + 50));
        wsm->setRoi_up(Coord(curPosition.x - 50, curPosition.y - 50));
        wsm->setDlastx(dlastx);
        wsm->setDlasty(dlasty);
        wsm->setLastx(lastx);
        wsm->setLasty(lasty);
        wsm->setKind(3);
    }

    wsm->setId(getParentModule()->getIndex());
    EV << " PARENT MODULE IS " << getParentModule()->getIndex();
    return wsm;
}
